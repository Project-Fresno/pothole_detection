#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class imageSubscriber(Node):
    def __init__(self):

        super().__init__("image_subscriber")

        self.depth_image = np.zeros((720, 1280), np.uint16)
        self.camera_info = CameraInfo()

        self.color_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.color_callback, 10
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/aligned_depth_to_color/camera_info",
            self.info_callback,
            10,
        )

        self.br = CvBridge()

        self.camera_info_pub = self.create_publisher(CameraInfo, "/cov_info", 10)

        self.lane_depth_publisher = self.create_publisher(Image, "lane_depth", 10)

    def info_callback(self, data):
        self.get_logger().info("Receiving camera info")
        self.camera_info = data

    def depth_callback(self, data):
        self.get_logger().info("Receiving depth frame")
        self.depth_image = self.br.imgmsg_to_cv2(data, "passthrough")
        # print(self.depth_image.shape)
        # cv2.imshow("original depth",self.depth_image)
        # cv2.waitKey(1)

    def color_callback(self, data):
        self.get_logger().info("Receiving color frame")

        self.color_image = self.br.imgmsg_to_cv2(data, "bgr8")
        # print("color", self.color_image.shape)

        self.hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        # print("hsv", self.hsv.shape)

        self.lower_white_hsv = (76, 70, 189)
        self.upper_white_hsv = (119, 95, 255)

        self.mask = cv2.inRange(self.hsv, self.lower_white_hsv, self.upper_white_hsv)
        # print("mask1", self.mask.shape)

        # self.masked = cv2.bitwise_and(self.hsv, self.hsv, mask=self.mask)
        # cv2.imshow("masked_image", self.masked)

        se = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_clean = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, se)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, se)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask_clean, connectivity=8
        )

        min_area = 500  # TUNE - Minimum area of object to keep
        filtered_mask = np.zeros_like(mask_clean)
        for i in range(1, num_labels):  # skip background (label 0)
            x, y, w, h, area = stats[i]
            if area >= min_area:
                filtered_mask[labels == i] = 255

        # cv2.imshow("pre-thinning", filtered_mask)

        # TRY WHEN USING POTHOLES TOO
        # contours, _ = cv2.findContours(
        #     filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        # )
        # skeleton = np.zeros_like(filtered_mask)
        # cv2.drawContours(skeleton, contours, -1, 255, 1)

        skeleton = cv2.ximgproc.thinning(filtered_mask)
        self.out_binary = skeleton
        # print(self.out_binary.shape)

        # cv2.imshow("mask", self.out_binary)
        self.lane_depth = cv2.bitwise_and(
            self.depth_image, self.depth_image, mask=self.out_binary
        )
        # cv2.waitKey(1)

        lane_depth_image = self.br.cv2_to_imgmsg(self.lane_depth)

        time = self.get_clock().now().to_msg()

        lane_depth_image.header.stamp = time

        lane_depth_image.header.frame_id = "camera_depth_optical_frame"

        self.camera_info.header.stamp = time

        self.lane_depth_publisher.publish(lane_depth_image)
        self.camera_info_pub.publish(self.camera_info)

        print("Lane Depth Published\n")


def main(args=None):

    rclpy.init(args=args)

    image_subscriber = imageSubscriber()

    rclpy.spin(image_subscriber)


if __name__ == "__main__":
    main()
