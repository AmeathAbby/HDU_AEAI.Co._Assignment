#!/usr/bin/env python3
"""Keep RGB-D generation active across short-lived PointCloud2 viewers.

The Noetic OpenNI Gazebo plugin disables the sensor when its last cloud
subscriber disconnects, even if RGB consumers remain. A persistent cloud
subscription prevents this. AnyMsg avoids unpacking millions of points.
"""
import rospy

if __name__ == '__main__':
    rospy.init_node('camera_stream_guard')
    cloud = rospy.Subscriber('~points', rospy.AnyMsg, lambda _: None,
                             queue_size=1, buff_size=2**24)
    rospy.loginfo('Keeping point cloud stream active: %s', cloud.resolved_name)
    rospy.spin()
