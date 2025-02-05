import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
import cv2 as cv
import cv2.aruco as aruco
import os
import numpy as np

# Used to convert OpenCV Mat type to ROS Image type
# NOTE: This may not be the most effective, we could turn the image into an JPEG string or even define a custom ROS data type.
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# A map of aruco dictionary strings to opencv.aruco enum values
aruco_dict_map = {
    "4x4_50": aruco.DICT_4X4_50,
    "4x4_100": aruco.DICT_4X4_100,
    "4x4_250": aruco.DICT_4X4_250,
    "4x4_1000": aruco.DICT_4X4_1000,
    "5x5_50": aruco.DICT_5X5_50,
    "5x5_100": aruco.DICT_5X5_100,
    "5x5_250": aruco.DICT_5X5_250,
    "5x5_1000": aruco.DICT_5X5_1000,
    "6x6_50": aruco.DICT_6X6_50,
    "6x6_100": aruco.DICT_6X6_100,
    "6x6_250": aruco.DICT_6X6_250,
    "6x6_1000": aruco.DICT_6X6_1000,
    "7x7_50": aruco.DICT_7X7_50,
    "7x7_100": aruco.DICT_7X7_100,
    "7x7_250": aruco.DICT_7X7_250,
    "7x7_1000": aruco.DICT_7X7_1000
}

class ArucoNode(Node):

    def __init__(self):
        super().__init__("aruco_node")
        # Declare the parameters for the node
        from rcl_interfaces.msg import ParameterDescriptor
        self.camera_config_file = self.declare_parameter('camera_config_file', 'cam.yml',
            ParameterDescriptor(
                description="The camera config .yml file that contains camera matrix, distance coefficients, and reprojection error",
                read_only=True
            )
        ).get_parameter_value().string_value
        self.write_video = self.declare_parameter('write_video', False,
            ParameterDescriptor(
                description="If set to True, it will create all output videos in the `/{video_dir}` directory",
                read_only=True
            )
        ).get_parameter_value().string_value
        self.video_dir = self.declare_parameter('video_dir', 'videos', 
           ParameterDescriptor(
                description="The name of the directory where video files are saved",
                read_only=True
            )
        ).get_parameter_value().string_value
        self.video_fps = self.declare_parameter('video_fps', 30, 
           ParameterDescriptor(
                description="The frames per second (FPS) of the output videos",
                read_only=True
           )
        ).get_parameter_value().integer_value
        # TODO: You should be able to change this parameter while the node is running and everything work fine
        self.ids_to_detect = self.declare_parameter('ids_to_detect', [1],
           ParameterDescriptor(
                description="The ids to detect in an image",
           )
        ).get_parameter_value().integer_array_value
        self.aruco_dict = self.declare_parameter('aruco_dict', "4x4_50", # correlates to URC standard dictionary
           ParameterDescriptor(
                description="This is the aruco dictionary number to assume aruco tags are from which come from 'cv2.aruco' predefined dictionary enum",
           )
        ).get_parameter_value().string_value
        self.marker_length = self.declare_parameter('marker_length', 0.175, # correlates to URC standard marker length
           ParameterDescriptor(
                description="This is the length (in meters) of the one side of the aruco marker you are tring to detect (not including white border).",
           )
        ).get_parameter_value().double_value

        # Camera calibration configuration
        (
         self.camera_mat,
         self.dist_coeffs,
         self.rep_error
        ) = self.read_camera_config_file()
        self.get_logger().info("Finished reading calibration information for camera")

        # Video writer creation
        self.write_video = self.get_parameter('write_video').get_parameter_value().bool_value
        if self.write_video:
            self.video_writer = None

            os.makedirs(self.video_dir, exist_ok=True)
            self.video_path = os.path.join(self.video_dir, "output_video.mp4")
            self.get_logger().info("Finished creating video writer for tracking output")

        # Aruco detector configuration
        self.detector_params = aruco.DetectorParameters() # TODO: Look into this
        self.tracker = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco_dict_map[self.aruco_dict]),
            self.detector_params
        )
        self.obj_points = np.array([ # Real world 3D coordinates of the marker corners
            [-self.marker_length / 2, 0,  self.marker_length / 2], # Top-left
            [ self.marker_length / 2, 0,  self.marker_length / 2], # Top-right
            [ self.marker_length / 2, 0, -self.marker_length / 2], # Bottom-right
            [-self.marker_length / 2, 0, -self.marker_length / 2], # Bottom-left
        ])
        self.get_logger().info("Finished creating aruco tracker")

        # Subscriber configuration
        self.subscription = self.create_subscription(
                Image, 'image', self.image_callback, 1)
        self.bridge = CvBridge()
        self.get_logger().info("Finished creating image subscriber")

        # Useful log information on startup
        self.get_logger().info(f"Set to detect the following aruco ids with marker length {self.marker_length}: {list(self.ids_to_detect)}")


    def read_camera_config_file(self):
        """Read the camera configuration file and return parameters"""
        
        # Try to open camera config file
        fs = cv.FileStorage(self.camera_config_file, cv.FILE_STORAGE_READ)
        if not fs.isOpened():
            self.get_logger().error(f"Error opening camrea config file at {camera_config_file}")
            exit(1)

        camera_mat = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        rep_error = fs.getNode("reproj_error").real()
        fs.release()

        return camera_mat, dist_coeffs, rep_error

    def image_callback(self, image_msg):
        """Process images received from topic"""
        try:
            # Convert ROS2 Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Initialize the video writer if activated
            if self.write_video and self.video_writer is None:
                frame_size = (cv_image.shape[1], cv_image.shape[0])
                self.video_writer = cv.VideoWriter(
                    self.video_path,
                    cv.VideoWriter_fourcc(*'mp4v'),
                    self.video_fps,
                    self.frame_size
                )
                self.get_logger().info(f"Video recording started: {self.video_path}")

            # Detect the marker ids
            marker_corners, marker_ids = self.detect_aruco_markers(cv_image)

            cv_detection_image = aruco.drawDetectedMarkers(cv_image, 
                                                           marker_corners,
                                                           marker_ids)

            # Calculate pose for each marker
            if (marker_ids is not None and len(marker_ids) != 0):
                for img_points in marker_corners:
                    retval, rvec, tvec = calculate_pose(img_points)



        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
        
    def detect_aruco_markers(self, image):
        """Given an image, return detected aruco markers and rejected markers (candidates for aruco markers)"""
        # Detect markers (corners and ids) and possible corners (rejected)
        (
         detected_marker_corners,
         detected_marker_ids,
         rejected_marker_ids
        ) = self.tracker.detectMarkers(image)
        return detected_marker_corners, detected_marker_ids

    def calculate_pose(self, img_points):
        """Given a set of image points (detected aruco corners), return the pose for the marker"""
        img_points = np.array(img_points, dtype=np.float32).reshape(4, 2)
        retval, rvec, tvec = cv.solvePnP(
            self.obj_points, img_points, self.camera_mat, self.dist_coeffs
        )
        return retval, rvec, tvec
        
    def close_video(self):
        """Close the video writer if it was activated"""
        if self.write_video:
            self.video_writer.release()
            self.get_logger().info(f"Video Saved: '{self.video_path}'")

def main(args=None):
    """Handle spinning up and destroying a node"""
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
