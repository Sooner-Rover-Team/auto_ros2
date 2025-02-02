import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
import cv2 as cv
import cv2.aruco as aruco
import os

# Used to convert OpenCV Mat type to ROS Image type
# NOTE: This may not be the most effective, we could turn the image into an JPEG string or even define a custom ROS data type.
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoNode(Node):

    def __init__(self):
        super().__init__("aruco_node")

        # Subscriber configuration
        self.subscription = self.create_subscription(
                Image, 'image', self.image_callback, 1)
        self.bridge = CvBridge()

        # Video writer configuration
        self.video_writer = None
        self.frame_size = None
        self.fps = 30
        self.video_dir = "videos"
        os.makedirs(self.video_dir, exist_ok=True)
        self.video_path = os.path.join(self.video_dir, "output_video.mp4") # TODO: Make this configureable

        # Aruco detector configuration
        self.ids_to_detect = [0] # TODO: Make this configurable
        self.dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # TODO: Make this configurable. By default, it's 4x4_50=0
        self.detector_params = aruco.DetectorParameters() # TODO: Look into this
        self.tracker = aruco.ArucoDetector(self.dict, self.detector_params)


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

            self.get_logger().info("Image captured")

            # TODO: Grayscaling/Image Processing

            # Detect the marker ids
            marker_corners, marker_ids = self.detect_aruco_markers(cv_image)
            self.get_logger().info(f"Found the follwing ids: {marker_ids}")

            cv_detection_image = aruco.drawDetectedMarkers(cv_image, 
                                                           marker_corners,
                                                           marker_ids)
            self.video_writer.write(cv_detection_image)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
        
    def detect_aruco_markers(self, image):
        # Detect markers (corners and ids) and possible corners (rejected)
        (
         detected_marker_corners,
         detected_marker_ids,
         rejected_marker_ids
        )= self.tracker.detectMarkers(image)
        return detected_marker_corners, detected_marker_ids
        
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
