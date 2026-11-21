#!/usr/bin/env python3
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_name = rospy.get_param('~color_name', '红色')
        self.ranges = rospy.get_param('~hsv_ranges')
        if not self.ranges:
            raise ValueError('hsv_ranges must not be empty')
        self.bounds = []
        for interval in self.ranges:
            lo, hi = interval['lower'], interval['upper']
            if (len(lo) != 3 or len(hi) != 3 or
                    any(not isinstance(v, int) for v in lo + hi) or
                    any(not 0 <= lo[i] <= hi[i] <= [179, 255, 255][i] for i in range(3))):
                raise ValueError('Invalid HSV bounds: {}'.format(interval))
            self.bounds.append((np.array(lo, np.uint8), np.array(hi, np.uint8)))
        self.min_area = float(rospy.get_param('~min_area', 300))
        self.min_ratio = float(rospy.get_param('~min_area_ratio', 0.001))
        kernel = rospy.get_param('~kernel_size', 3)
        self.timeout = float(rospy.get_param('~image_timeout', 2.0))
        self.log_period = float(rospy.get_param('~log_period', 1.0))
        if (not np.isfinite([self.min_area, self.min_ratio, self.timeout, self.log_period]).all()
                or self.min_area <= 0 or not 0 <= self.min_ratio <= 1 or self.timeout <= 0
                or self.log_period <= 0 or not isinstance(kernel, int) or kernel < 1 or kernel % 2 != 1):
            raise ValueError('Invalid detection/timeout/kernel parameters')
        self.kernel = np.ones((kernel, kernel), np.uint8)
        self.lock = threading.Lock()
        self.last_valid = None
        self.previous = None
        self.last_log = 0.0
        self.pub = rospy.Publisher('~detected', Bool, queue_size=1, latch=False)
        self.target_pub = rospy.Publisher('~target_x', Float32, queue_size=1, latch=False)
        self.debug_pub = rospy.Publisher('~debug_image', Image, queue_size=1)
        self.debug = rospy.get_param('~publish_debug_image', True)
        self.sub = rospy.Subscriber('~image', Image, self.on_image, queue_size=1, buff_size=2**24)
        # Wall time watchdog works even when Gazebo simulation time is paused.
        self.stop = threading.Event()
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()
        rospy.on_shutdown(self.stop.set)
        rospy.loginfo('%s detector: input=%s output=%s', self.color_name, self.sub.resolved_name, self.pub.resolved_name)

    def detect(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.bounds:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        threshold = max(self.min_area, self.min_ratio * image.shape[0] * image.shape[1])
        valid = [c for c in contours if cv2.contourArea(c) >= threshold]
        largest = max(valid, key=cv2.contourArea) if valid else None
        area = cv2.contourArea(largest) if largest is not None else 0.0
        # Tall, upright contour: the pillar, not the flat pink dots on the floor.
        tall, tall_area = None, 0.0
        for c in valid:
            _, _, w, h = cv2.boundingRect(c)
            if h >= w and cv2.contourArea(c) > tall_area:
                tall, tall_area = c, cv2.contourArea(c)
        return largest is not None, area, largest, tall

    def on_image(self, msg):
        # Serialize callback and watchdog, so an expired result cannot overtake a fresh one.
        with self.lock:
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                detected, area, contour, tall = self.detect(image)
            except (CvBridgeError, cv2.error, ValueError) as exc:
                self.last_valid = None
                self.pub.publish(Bool(data=False))
                self.target_pub.publish(Float32(data=float('nan')))
                rospy.logerr_throttle(2.0, 'Invalid camera image; detected=false: {}'.format(exc))
                return
            now = time.monotonic()
            self.last_valid = now
            self.pub.publish(Bool(data=bool(detected)))
            bbox = cv2.boundingRect(contour) if detected else None
            if tall is not None:
                x, y, w, h = cv2.boundingRect(tall)
                # Normalized horizontal offset of the upright pink pillar: 0 is center.
                offset = (x + w / 2.0 - image.shape[1] / 2.0) / (image.shape[1] / 2.0)
                self.target_pub.publish(Float32(data=float(offset)))
            else:
                self.target_pub.publish(Float32(data=float('nan')))
            if detected != self.previous or now - self.last_log >= self.log_period:
                rospy.loginfo('%s检测: %s | detected=%s | largest_area=%.0f px',
                              self.color_name, '存在目标' if detected else '未发现目标', detected, area)
                self.last_log = now
                self.previous = detected
            if self.debug and self.debug_pub.get_num_connections():
                preview = image.copy()
                if bbox is not None:
                    x, y, w, h = bbox
                    cv2.rectangle(preview, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(preview, 'TARGET: {}'.format(detected), (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                debug_msg = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)

    def watch(self):
        while not self.stop.wait(0.2):
            with self.lock:
                if self.last_valid is None or time.monotonic() - self.last_valid > self.timeout:
                    self.pub.publish(Bool(data=False))
                    self.target_pub.publish(Float32(data=float('nan')))
                    self.previous = None
                    rospy.logwarn_throttle(2.0, 'Camera image missing/stale; detected=false (input unavailable)')


if __name__ == '__main__':
    rospy.init_node('color_detector')
    try:
        detector = ColorDetector()
        rospy.spin()
    except (ValueError, KeyError, TypeError) as error:
        rospy.logfatal('Color detector configuration error: %s', error)
        raise SystemExit(1)
