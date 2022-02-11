#!/usr/bin/env python3

import rospy
import cv2
import message_filters
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped


class DrawGaze:
    def __init__(self):
        self.bridge = CvBridge()
        self.fpv_image_sub = message_filters.Subscriber(
            "pupil/fpv_image/compressed", CompressedImage, queue_size=10
        )
        self.gaze_pt_sub = message_filters.Subscriber(
            "pupil/gaze_pt", PointStamped, queue_size=10
        )
        self.gaze_img_pub = rospy.Publisher("gaze_image", Image, queue_size=10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.fpv_image_sub, self.gaze_pt_sub], queue_size=1, slop=1
        )
        self.ts.registerCallback(self.callback)

    def callback(self, fpv_image, gaze_pt):

        np_arr = np.frombuffer(fpv_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        # world_img = self.bridge.imgmsg_to_cv2(fpv_image, desired_encoding='8UC3')
        cv2.circle(
            image_np,
            (int(gaze_pt.point.x), int(gaze_pt.point.y)),
            40,
            (0, 0, 255),
            4,
        )
        # world_img = np.uint8(world_img)
        image_message = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
        image_message.header.stamp = fpv_image.header.stamp
        self.gaze_img_pub.publish(image_message)


if __name__ == "__main__":
    rospy.init_node("draw_gaze", anonymous=True)
    DrawGaze()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
