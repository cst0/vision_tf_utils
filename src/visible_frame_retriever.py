#!/usr/bin/env python3

import rospy
import tf2_ros
import yaml

from math import acos, sqrt, atan
from image_geometry.cameramodels import PinholeCameraModel

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import Image, CameraInfo


class PinholeCameraModelWithFov(PinholeCameraModel):
    """
    Extension of pinhole camera model to allow calculation of the field of view.
    Should be covered by https://github.com/ros-perception/vision_opencv/pull/428
    but this is not yet merged.
    """

    def fovX(self):
        K = self.fullIntrinsicMatrix()
        assert K is not None
        alphax = K[0, 0]
        return 2 * atan(self.width / (2 * alphax))

    def fovY(self):
        K = self.fullIntrinsicMatrix()
        assert K is not None
        alphay = K[1, 1]
        return 2 * atan(self.height / (2 * alphay))


class VisibleFrameRetriever:
    """
    Retrieves visible frames, and makes the output available as a service.
    """

    def __init__(self):
        self.tf_buff = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buff)

        self.srv = rospy.Service("evaluate", Trigger, self.run)

        self.sub_camerainfo = rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.cb_camerainfo
        )
        self.sub_image = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.cb_image
        )

        self.image = Image()
        self.cameramodel = PinholeCameraModelWithFov()
        self.transforms_from_cameraframe = {}

    def cb_camerainfo(self, msg: CameraInfo):
        self.cameramodel.fromCameraInfo(msg)

    def cb_image(self, msg):
        self.image = msg

    def get_tf_links(self):
        return [*(yaml.full_load(self.tf_buff.all_frames_as_yaml()).keys())]

    def run(self, trig: TriggerRequest):
        del trig
        links = self.get_tf_links()
        cf = self.cameramodel.tfFrame()
        self.transforms_from_cameraframe = {}

        for link in links:
            self.transforms_from_cameraframe[link] = self.tf_buff.lookup_transform(
                cf, link, rospy.Time()
            )
        horizontal_angles = {}
        vertical_angles = {}
        in_frame = []
        for link in links:
            x = self.transforms_from_cameraframe[link].transform.translation.x
            y = self.transforms_from_cameraframe[link].transform.translation.y
            z = self.transforms_from_cameraframe[link].transform.translation.z

            # recall per REP 108 (?) that x is forward, y is left, z is up (relative to frame).
            # so finding the horizontal angle, where x = forward = 0 is between x frame and (x,y),
            # while finding the vertical angle is between x and (x,z).
            # doing this via scalar projection: because we're doing this relative to a known
            # frame, we know that the projection length is equal to the x component,
            # and so |xy|cos(theta) -> theta = acos(x / sqrt(x^2+y^2)).
            # also adding a safety check for 0-degree transforms, which are very common
            # in static tf links.
            horizontal_angles[link] = (
                acos(x / sqrt(x * x + y * y)) if (x * x + y * y) != 0 else 0
            )
            # same deal for vertical
            vertical_angles[link] = (
                acos(x / sqrt(x * x + z * z)) if (x * x + z * z) != 0 else 0
            )

            # getting fov. Dividing by 2 because previous operation gets angle from center,
            # but this is giving total angle.
            fovx = (self.cameramodel.fovX()) / 2
            fovy = (self.cameramodel.fovY()) / 2

            if horizontal_angles[link] < fovx and vertical_angles[link] < fovy:
                in_frame.append(link)

        return TriggerResponse(success=True, message=str(in_frame))


def main():
    rospy.init_node("visible_frame_retriever")
    vfr = VisibleFrameRetriever()
    rospy.spin()


if __name__ == "__main__":
    main()
