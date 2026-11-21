#!/usr/bin/env python3
"""SITL-only ENU mission. Faults stop setpoints and leave recovery to PX4 failsafe.

No automatic retry/re-arm after a fault. Service calls never block the publisher.
All watchdogs use monotonic wall time, including when /clock stops.
"""
import copy
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


def validate_config(raw):
    c = copy.deepcopy(raw)
    c.setdefault("state_timeout", 3.0)
    if c.get("frame") != "ENU":
        raise ValueError("Only ENU coordinates are accepted")
    keys = ("frequency", "tolerance", "yaw_tolerance", "waypoint_timeout",
            "min_altitude", "max_altitude", "ground_min_altitude",
            "min_x", "max_x", "min_y", "max_y", "telemetry_timeout", "state_timeout",
            "connection_timeout", "activation_timeout", "service_timeout")
    for key in keys:
        c[key] = float(c[key])
        if not math.isfinite(c[key]):
            raise ValueError("Non-finite config: " + key)
    for key in ("tolerance", "yaw_tolerance", "waypoint_timeout",
                "telemetry_timeout", "state_timeout", "connection_timeout", "activation_timeout",
                "service_timeout"):
        if c[key] <= 0:
            raise ValueError("Expected positive " + key)
    if c["frequency"] != 20.0:
        raise ValueError("This controller requires 20 Hz")
    if not (c["ground_min_altitude"] <= c["min_altitude"] < c["max_altitude"]):
        raise ValueError("Invalid altitude limits")
    if c["min_x"] >= c["max_x"] or c["min_y"] >= c["max_y"]:
        raise ValueError("Invalid horizontal limits")
    n = c["prestream_count"]
    if isinstance(n, bool) or int(n) != n or n < 40:
        raise ValueError("prestream_count must be an integer >= 40 (2 seconds)")
    c["prestream_count"] = int(n)
    if not isinstance(c.get("waypoints"), list) or not c["waypoints"]:
        raise ValueError("Missing waypoints")
    for w in c["waypoints"]:
        for key in ("x", "y", "z", "yaw", "hold"):
            w[key] = float(w[key])
            if not math.isfinite(w[key]):
                raise ValueError("Non-finite waypoint " + key)
        if not (c["min_x"] <= w["x"] <= c["max_x"] and
                c["min_y"] <= w["y"] <= c["max_y"] and
                c["min_altitude"] <= w["z"] <= c["max_altitude"]):
            raise ValueError("Waypoint outside configured envelope")
        if w["hold"] < 0 or w["hold"] >= c["waypoint_timeout"]:
            raise ValueError("Hold must be nonnegative and less than timeout")
    return c


def mission_config(raw, mission, square_side):
    """Build the BaseMap bonus route in ENU relative to the Iris spawn."""
    if mission == "basic":
        return validate_config(raw)
    if mission not in ("square", "avoid"):
        raise ValueError("mission must be basic, square or avoid")
    side = float(square_side)
    if not 1.25 <= side <= 1.70:
        raise ValueError("BaseMap square_side must be between 1.25 and 1.70 m")
    config = copy.deepcopy(raw)
    waypoints = config["waypoints"][:1]
    cruise = 2.65  # Clears the original 2 m columns and interior walls.

    def point(name, x, y, z, hold=1.5):
        return dict(name=name, x=x, y=y, z=z, yaw=0.0, hold=hold)

    if mission == "avoid":
        # The static obstacle is centered at ENU (0.65, 0) and is 2.75 m tall.
        # Climb before the crossing, then descend only beyond its footprint.
        config["max_altitude"] = 3.65
        waypoints.extend([
            point("OBSTACLE_CLIMB", 0, 0, 3.30),
            point("OBSTACLE_CROSS", side, 0, 3.30),
        ])
    else:
        config["max_altitude"] = 3.0
        waypoints.append(point("SQUARE_CLIMB", 0, 0, cruise))

    waypoints.extend([
        point("SQUARE_P1", side, 0, cruise),
        point("SQUARE_P2", side, side, cruise),
        point("SQUARE_P3", 0, side, cruise),
        point("SQUARE_P4", 0, 0, cruise),
    ])
    config["waypoints"] = waypoints
    return validate_config(config)


def pose_values(msg):
    p, q = msg.pose.position, msg.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite local pose")
    norm = sum(v * v for v in values[3:])
    if abs(norm - 1.0) > 0.05:
        raise ValueError("Invalid local pose quaternion")
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return p.x, p.y, p.z, yaw


def reached(values, w, c):
    x, y, z, yaw = values
    distance = math.sqrt((x-w["x"])**2 + (y-w["y"])**2 + (z-w["z"])**2)
    yaw_error = abs(math.atan2(math.sin(yaw-w["yaw"]), math.cos(yaw-w["yaw"])))
    return distance <= c["tolerance"] and yaw_error <= c["yaw_tolerance"]


class TrajectoryFollower:
    def __init__(self):
        self.c = mission_config(rospy.get_param("~trajectory"),
                                rospy.get_param("~mission", "basic"),
                                rospy.get_param("~square_side", 1.40))
        self.lock = threading.RLock()
        self.state = State()
        self.pose = None
        self.state_received = self.pose_received = 0.0
        self.last_pose_stamp = None
        self.target = None
        self.fault = None
        self.require_offboard = self.require_armed = False
        self.arm_requested = False
        self.sent = 0
        self.stop = threading.Event()
        self.pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=1)
        self.sub_pose = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=1)
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.thread = None

    def state_cb(self, msg):
        with self.lock:
            self.state, self.state_received = msg, time.monotonic()

    def pose_cb(self, msg):
        with self.lock:
            # Replayed/unchanged stamped poses do not refresh the watchdog.
            stamp = msg.header.stamp.to_sec()
            if not math.isfinite(stamp) or stamp <= 0:
                return
            if self.last_pose_stamp is not None and stamp <= self.last_pose_stamp:
                return
            self.last_pose_stamp = stamp
            self.pose, self.pose_received = msg, time.monotonic()

    def health(self):
        now = time.monotonic()
        if not self.state.connected:
            return "FCU reports disconnected"
        state_age = now-self.state_received
        if state_age > self.c["state_timeout"]:
            return "FCU state stale: age={:.3f}s exceeds {:.3f}s".format(state_age, self.c["state_timeout"])
        if self.pose is None or now-self.pose_received > self.c["telemetry_timeout"]:
            return "Local pose missing/stale (or /clock stopped)"
        age = rospy.Time.now().to_sec() - self.pose.header.stamp.to_sec()
        if age < -0.2 or age > self.c["telemetry_timeout"]:
            return "Local pose timestamp outside freshness window"
        try:
            x, y, z, _ = pose_values(self.pose)
        except ValueError as exc:
            return str(exc)
        if not (self.c["min_x"] <= x <= self.c["max_x"] and
                self.c["min_y"] <= y <= self.c["max_y"] and
                self.c["ground_min_altitude"] <= z <= self.c["max_altitude"]):
            return ("Measured position outside safety envelope: x={:.2f}, y={:.2f}, z={:.2f}; "
                    "limits x=[{}, {}], y=[{}, {}], z=[{}, {}]".format(
                        x, y, z, self.c["min_x"], self.c["max_x"],
                        self.c["min_y"], self.c["max_y"],
                        self.c["ground_min_altitude"], self.c["max_altitude"]))
        if self.require_offboard and self.state.mode != "OFFBOARD":
            return "OFFBOARD lost; no automatic re-entry"
        if self.require_armed and not self.state.armed:
            return "Unexpected disarm; no automatic re-arm"
        if self.state.armed and not self.arm_requested:
            return "Vehicle armed before controller authorization"
        return None

    def startup_ready(self):
        with self.lock:
            reason = self.health()
            self.last_ready_error = reason
            now = time.monotonic()
            if reason and now-getattr(self, "last_ready_report", float("-inf")) >= 5:
                rospy.logwarn("Waiting for startup readiness: %s", reason)
                self.last_ready_report = now
            return reason is None

    def abort(self, reason):
        with self.lock:
            if self.fault is None:
                self.fault = reason
                rospy.logerr("Mission aborted: %s. Setpoints stopped; PX4 failsafe applies.", reason)
            self.target = None
            self.stop.set()

    def stream(self):
        try:
            self._stream_loop()
        except Exception as exc:
            self.abort("Setpoint publisher failed: " + str(exc))

    def _stream_loop(self):
        period = 1.0 / self.c["frequency"]
        previous = time.monotonic()
        while not self.stop.is_set() and not rospy.is_shutdown():
            with self.lock:
                reason = self.health()
                if time.monotonic() - previous > 0.25:
                    reason = "Setpoint publisher scheduling gap > 0.25 s"
                if reason:
                    self.abort(reason)
                    return
                previous = time.monotonic()
                w = self.target
                if w is not None:
                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "map"
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = w["x"], w["y"], w["z"]
                    msg.pose.orientation.z = math.sin(w["yaw"]/2)
                    msg.pose.orientation.w = math.cos(w["yaw"]/2)
                    self.pub.publish(msg)
                    self.sent += 1
            self.stop.wait(period)

    def wait_until(self, predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and not self.stop.is_set():
            with self.lock:
                if predicate():
                    return True
            if time.monotonic() >= deadline:
                self.abort(reason() if callable(reason) else reason)
                return False
            self.stop.wait(0.02)
        return False

    def call(self, proxy, **kwargs):
        # A hung TCPROS service cannot interrupt the 20 Hz stream or block shutdown.
        done, result = threading.Event(), []
        def worker():
            try:
                result.append(proxy(**kwargs))
            except Exception as exc:
                result.append(exc)
            finally:
                done.set()
        threading.Thread(target=worker, daemon=True).start()
        if not self.wait_until(done.is_set, self.c["service_timeout"], "MAVROS service timed out"):
            return None
        if isinstance(result[0], Exception):
            self.abort("MAVROS service failed: " + str(result[0]))
            return None
        return result[0]

    def run(self):
        if not rospy.get_param("~enable_control", False):
            rospy.logwarn("Control disabled. Validate SITL, then explicitly set enable_control:=true.")
            return
        if not self.wait_until(self.startup_ready,
                               self.c["connection_timeout"],
                               lambda: "Startup readiness timeout: " + str(self.last_ready_error)):
            return
        for service in ("/mavros/set_mode", "/mavros/cmd/arming"):
            rospy.wait_for_service(service, timeout=self.c["service_timeout"])
        with self.lock:
            self.target = self.c["waypoints"][0]
        self.thread = threading.Thread(target=self.stream, daemon=True)
        self.thread.start()
        if not self.wait_until(lambda: self.sent >= self.c["prestream_count"],
                               self.c["prestream_count"]/20.0 + 5, "Prestream timeout"):
            return
        response = self.call(self.set_mode, custom_mode="OFFBOARD")
        if response is None:
            return
        if not response.mode_sent:
            self.abort("OFFBOARD rejected")
            return
        if not self.wait_until(lambda: self.state.mode == "OFFBOARD",
                               self.c["activation_timeout"], "OFFBOARD not confirmed in /mavros/state"):
            return
        with self.lock:
            self.require_offboard = True
            if self.health():
                self.abort(self.health())
                return
            self.arm_requested = True
        response = self.call(self.arm, value=True)
        if response is None:
            return
        if not response.success:
            self.abort("Arming rejected")
            return
        if not self.wait_until(lambda: self.state.armed,
                               self.c["activation_timeout"], "Arming not confirmed in /mavros/state"):
            return
        with self.lock:
            self.require_armed = True
        for w in self.c["waypoints"]:
            with self.lock:
                self.target = w
            rospy.loginfo("Waypoint %s", w.get("name", "unnamed"))
            deadline, held_since = time.monotonic() + self.c["waypoint_timeout"], None
            while not self.stop.is_set() and not rospy.is_shutdown():
                with self.lock:
                    inside = reached(pose_values(self.pose), w, self.c)
                now = time.monotonic()
                held_since = (now if held_since is None else held_since) if inside else None
                if held_since is not None and now-held_since >= w["hold"]:
                    break
                if now >= deadline:
                    self.abort("Waypoint/continuous hold timeout: " + w.get("name", "unnamed"))
                    return
                self.stop.wait(0.02)
            if self.stop.is_set() or rospy.is_shutdown():
                return
        rospy.loginfo("Trajectory complete; holding final target until shutdown or a safety fault")
        while not rospy.is_shutdown() and not self.stop.is_set():
            self.stop.wait(0.05)

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=1)


if __name__ == "__main__":
    rospy.init_node("trajectory_follower")
    follower = None
    try:
        follower = TrajectoryFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Trajectory follower failed: %s", exc)
        raise
    finally:
        if follower is not None:
            follower.close()
