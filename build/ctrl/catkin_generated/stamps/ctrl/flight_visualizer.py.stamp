#!/usr/bin/env python3
"""RViz-only actual aircraft and path. No file recording or flight commands."""
import copy
import math
import threading
import time
from collections import deque


def valid_pose(msg):
    p,q=msg.pose.position,msg.pose.orientation
    values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
    return (bool(msg.header.frame_id) and msg.header.stamp.to_sec()>0 and
            all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:])-1)<0.05)


class Visualizer:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import PoseStamped, Point, TransformStamped
        from tf2_ros import TransformBroadcaster
        from nav_msgs.msg import Path
        from visualization_msgs.msg import Marker, MarkerArray
        self.ros,self.Point,self.Path,self.Marker,self.Array=rospy,Point,Path,Marker,MarkerArray
        self.Transform=TransformStamped
        self.tf=TransformBroadcaster()
        self.lock=threading.RLock()
        self.pose=None
        self.received=0
        self.last_visual=0
        limit=int(rospy.get_param("~max_path_points",0))
        if limit<0:raise ValueError("max_path_points must be >=0")
        self.points=deque(maxlen=limit or None)
        self.markers=rospy.Publisher("/ctrl/aircraft_markers",MarkerArray,queue_size=1,latch=True)
        self.path=rospy.Publisher("/ctrl/flight_path",Path,queue_size=1,latch=True)
        self.sub=rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.callback,queue_size=100)
        self.timer=rospy.Timer(rospy.Duration(0.2),self.update)
        rospy.loginfo("RViz aircraft/path visualization enabled; waiting for MAVROS pose. No CSV or flight commands.")

    def callback(self,msg):
        if not valid_pose(msg):
            self.ros.logwarn_throttle(5,"Visualization skipped invalid pose/frame/quaternion")
            return
        with self.lock:
            now=time.monotonic()
            if self.pose and (msg.header.frame_id!=self.pose.header.frame_id or
                             msg.header.stamp.to_sec()<self.pose.header.stamp.to_sec()):
                self.points.clear()
                self.ros.logwarn("Pose frame/time changed: cleared visual flight history")
            self.pose=copy.deepcopy(msg)
            self.received=now
            if not self.points:
                append=True
            else:
                a,b=self.points[-1].pose.position,msg.pose.position
                distance=math.sqrt((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2)
                append=(now-self.last_visual>=0.1 and distance>=0.03) or now-self.last_visual>=1
            if append:
                self.points.append(copy.deepcopy(msg))
                self.last_visual=now

    def aircraft(self,pose):
        M=self.Marker
        array=self.Array()
        def marker(idx,kind,color):
            m=M();m.header=copy.deepcopy(pose.header);m.ns="aircraft";m.id=idx;m.type=kind;m.action=M.ADD
            m.pose=copy.deepcopy(pose.pose)
            m.color.r,m.color.g,m.color.b,m.color.a=color
            return m
        body=marker(0,M.CUBE,(0.15,0.55,1.,1.));body.scale.x=.24;body.scale.y=.16;body.scale.z=.08
        arms=marker(1,M.LINE_LIST,(.8,.8,.85,1.));arms.scale.x=.035
        arms.points=[self.Point(-.25,-.25,0),self.Point(.25,.25,0),self.Point(-.25,.25,0),self.Point(.25,-.25,0)]
        rotors=marker(2,M.SPHERE_LIST,(.15,1.,.4,1.));rotors.scale.x=.2;rotors.scale.y=.2;rotors.scale.z=.025
        rotors.points=[self.Point(x,y,.035) for x,y in [(-.25,-.25),(.25,.25),(-.25,.25),(.25,-.25)]]
        nose=marker(3,M.ARROW,(1.,.2,.1,1.));nose.scale.x=.025;nose.scale.y=.07;nose.scale.z=.08
        nose.points=[self.Point(0,0,.09),self.Point(.4,0,.09)]
        array.markers=[body,arms,rotors,nose]
        return array

    def update(self,event):
        with self.lock:
            if self.pose is None:
                self.ros.logwarn_throttle(5,"No MAVROS pose: start MAVROS on ROS_MASTER_URI=http://127.0.0.1:11321")
                return
            if time.monotonic()-self.received>2:
                self.ros.logwarn_throttle(5,"MAVROS pose stale; aircraft display shows last received position")
            transform=self.Transform()
            transform.header=copy.deepcopy(self.pose.header)
            transform.child_frame_id="ctrl_aircraft"
            pos=self.pose.pose.position
            transform.transform.translation.x=pos.x
            transform.transform.translation.y=pos.y
            transform.transform.translation.z=pos.z
            transform.transform.rotation=copy.deepcopy(self.pose.pose.orientation)
            self.tf.sendTransform(transform)
            self.markers.publish(self.aircraft(self.pose))
            path=self.Path();path.header=copy.deepcopy(self.pose.header);path.poses=list(self.points)
            self.path.publish(path)


def main():
    import rospy
    rospy.init_node("flight_visualizer")
    node=Visualizer()
    rospy.spin()


if __name__=="__main__":main()
