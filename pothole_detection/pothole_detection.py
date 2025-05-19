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

        self.depth_image = np.zeros((480, 640), np.uint16)
        self.lane_depth = np.zeros((480, 640), np.uint16)

        self.color_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.color_callback, 10
        )  # '/camera/color/image_raw'

        self.depth_sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )  # 'camera/depth/image_rect_raw'

        self.info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/aligned_depth_to_color/camera_info",
            self.info_callback,
            10,
        )  # '/camera/depth/camera_info'

        self.br = CvBridge()
        self.camera_info_pub = self.create_publisher(CameraInfo, "/cov_info", 10)
        self.pothole_depth_publisher = self.create_publisher(
            Image, "pothole_depth", 10
        )  # origianlly 'output_depth
        self.lane_depth_publisher = self.create_publisher(Image, "lane_depth", 10)

        self.timer = self.create_timer(
            0.001, self.timer_callback  # publishing every 0.1 second
        )

        self.camera_info = CameraInfo()

        self.pothole_depth = np.zeros((720, 1280), np.uint16)
        self.lanes = np.zeros((720, 1280), dtype="uint8")

    def info_callback(self, data):
        self.get_logger().info("Receiving camera info")
        self.camera_info = data

    def timer_callback(self):
        pothole_depth_image = self.br.cv2_to_imgmsg(self.pothole_depth)
        lane_depth_image = self.br.cv2_to_imgmsg(self.lane_depth)

        time = self.get_clock().now().to_msg()

        pothole_depth_image.header.stamp = time
        lane_depth_image.header.stamp = time

        pothole_depth_image.header.frame_id = "camera_link"
        lane_depth_image.header.frame_id = "camera_link"

        self.camera_info.header.stamp = time

        self.pothole_depth_publisher.publish(pothole_depth_image)
        self.lane_depth_publisher.publish(lane_depth_image)
        self.camera_info_pub.publish(self.camera_info)

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

        self.lower_white_hsv = (0, 0, 207)  # 0,0,200          0,0,168
        self.upper_white_hsv = (179, 125, 255)  # 145,60,255       172,111,255

        self.mask = cv2.inRange(self.hsv, self.lower_white_hsv, self.upper_white_hsv)
        # print("mask1", self.mask.shape)

        # self.masked = cv2.bitwise_and(self.hsv, self.hsv, mask=self.mask)
        # cv2.imshow("masked_image", self.masked)

        se = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_clean = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, se)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, se)

        # self.out_gray = cv2.divide(self.blurred, bg, scale=255)

        # _, self.out_binary = cv2.threshold(
        #     self.out_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        # )

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask_clean, connectivity=8
        )

        min_area = 1500  # Minimum area of object to keep
        filtered_mask = np.zeros_like(mask_clean)
        for i in range(1, num_labels):  # skip background (label 0)
            x, y, w, h, area = stats[i]
            if area >= min_area:
                filtered_mask[labels == i] = 255

        self.out_binary = filtered_mask

        # cv2.imshow("mask", self.out_binary)
        self.lane_depth = cv2.bitwise_and(
            self.depth_image, self.depth_image, mask=self.out_binary
        )
        print("Lane Depth Published\n")

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    image_subscriber = imageSubscriber()

    rclpy.spin(image_subscriber)


if __name__ == "__main__":
    main()
