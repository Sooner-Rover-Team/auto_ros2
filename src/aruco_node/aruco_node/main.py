import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
import cv2 as cv
import os

# Used to convert OpenCV Mat type to ROS Image type
# NOTE: This may not be the most effective, we could turn the image into an JPEG string or even define a custom ROS data type.
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rosbag2_py

class ArucoNode(Node):

    def __init__(self):
        super().__init__("aruco_node")

        self.subscription = self.create_subscription(
                Image, 'image', self.image_callback, 1)
        self.bridge = CvBridge()

        self.video_writer = None
        self.frame_size = None
        self.fps = 30

        self.video_dir = "videos"
        os.makedirs(self.video_dir, exist_ok=True)
        self.video_path = os.path.join(self.video_dir, "output_video.mp4")

    def image_callback(self, image_msg):
        try:
            # Convert ROS2 Image to OpenCv format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            if self.video_writer is None:
                self.frame_size = (cv_image.shape[1], cv_image.shape[0])
                self.video_writer = cv.VideoWriter(
                    self.video_path,
                    cv.VideoWriter_fourcc(*'mp4v'),
                    self.fps,
                    self.frame_size
                )
                self.get_logger().info(f"Video recording started: {self.video_path}")

            self.video_writer.write(cv_image)
            self.get_logger().info("Image captured")
            

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def close_video(self):
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info(f"Video Saved: {self.video_path}")

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()

    try:
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        pass
    finally: # Ensure that the video writer closes properly
        aruco_node.close_video()

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        aruco_node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
