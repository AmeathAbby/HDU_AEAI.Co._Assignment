#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node('camera_stream_guard')
    cloud = rospy.Subscriber('~points', rospy.AnyMsg, lambda _: None,
                             queue_size=1, buff_size=2**24)
    rospy.loginfo('Keeping point cloud stream active: %s', cloud.resolved_name)
    rospy.spin()
