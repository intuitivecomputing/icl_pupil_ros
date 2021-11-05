#!/usr/bin/env python3
# Thanks to Gopika's work in https://github.com/intuitivecomputing/demo_data_processing/blob/master/scripts/gaze_data_pub.py

import numpy as np

import cv2
import ndsi

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, PointStamped


class GazeTracker:
    def __init__(self, draw_gaze=True):
        self.draw_gaze = draw_gaze
        self.sensors = {}
        self.sensor_types = ["video", "gaze"]
        self.cv_bridge = CvBridge()
        self.fpv_img_pub = rospy.Publisher(
            "fpv_image/compressed", CompressedImage, queue_size=10
        )
        self.gaze_pub = rospy.Publisher("gaze_pt", PointStamped, queue_size=10)

        # Start auto-discovery of Pupil Invisible Companion devices
        self.network = ndsi.Network(
            formats={ndsi.DataFormat.V4},
            callbacks=(self.network_event_callback,),
        )

    def __enter__(self):
        rospy.loginfo("Starting ndsi connection...")
        self.network.start()
        return self

    def __exit__(self, *args):
        rospy.loginfo("Stopping ndsi connection...")
        self.network.stop()

    def network_event_callback(self, network, event):
        # Handle gaze sensor attachment
        if (
            event["subject"] == "attach"
            and event["sensor_type"] in self.sensor_types
        ):
            # Create new sensor, start data streaming,
            # and request current configuration
            sensor = network.sensor(event["sensor_uuid"])
            sensor.set_control_value("streaming", True)
            sensor.refresh_controls()

            # Save sensor s.t. we can fetch data from it in main()
            self.sensors[event["sensor_uuid"]] = sensor
            rospy.loginfo(f"Added sensor {sensor}...")

        # Handle gaze sensor detachment
        if (
            event["subject"] == "detach"
            and event["sensor_uuid"] in self.sensors
        ):
            # Known sensor has disconnected, remove from list
            self.sensors[event["sensor_uuid"]].unlink()
            del self.sensors[event["sensor_uuid"]]
            rospy.loginfo(f"Removed sensor {event['sensor_uuid']}...")

    def update(self):
        fpv_img = None  # np.zeros((1088, 1080, 3))
        gaze_point = None  # (0, 0)
        # Check for recently connected/disconnected devices
        if self.network.has_events:
            self.network.handle_event()

        # Iterate over all connected devices
        for sensor in self.sensors.values():

            # We only consider gaze and video
            if sensor.type not in self.sensor_types:
                continue

            # Fetch recent sensor configuration changes,
            # required for pyndsi internals
            while sensor.has_notifications:
                sensor.handle_notification()

            # Fetch recent gaze data
            for data in sensor.fetch_data():
                if data is None:
                    continue

                if sensor.name == "PI world v1":
                    fpv_img = data.bgr

                elif sensor.name == "Gaze":
                    gaze_point = Point(int(data[0]), int(data[1]), 0.0)

        if gaze_point is not None:
            gaze_msg = PointStamped()
            gaze_msg.header = Header()
            gaze_msg.header.stamp = rospy.Time.now()
            gaze_msg.point = gaze_point
            self.gaze_pub.publish(gaze_msg)

        if fpv_img is not None:
            if self.draw_gaze and gaze_point is not None:
                # Draw gaze overlay onto world video frame
                cv2.circle(
                    fpv_img, (gaze_point.x, gaze_point.y), 40, (0, 0, 255), 4
                )
            try:
                image_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
                    fpv_img, dst_format="jpg"
                )
                image_msg.header = Header()
                image_msg.header.stamp = rospy.Time.now()
                self.fpv_img_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node("gaze_tracker_node")
    rate = rospy.Rate(10)

    draw_gaze = rospy.get_param("~draw_gaze")
    with GazeTracker(draw_gaze=draw_gaze) as tracker:
        while tracker.network.running and not rospy.is_shutdown():
            tracker.update()
            rate.sleep()
