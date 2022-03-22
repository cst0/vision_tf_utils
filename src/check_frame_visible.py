#!/usr/bin/env python3

import rospy
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo

class FrameVisibilityChecker:
    def __init__(self):
        self.tf_buff = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buff)
        rospy.loginfo(self.tf_buff.all_frames_as_yaml())


def main():
    rospy.init_node('check_frame_visible')
    fvc = FrameVisibilityChecker()
    rospy.spin()


if __name__ == "__main__":
    main()
