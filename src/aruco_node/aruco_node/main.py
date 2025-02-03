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

class ArucoNode(Node):

    def __init__(self):
        super().__init__("aruco_node")
        # Parameters for class
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('camera_config_file', 'cam.yml',
            ParameterDescriptor(description="The camera config .yml file that contains camera matrix, distance coefficients, and reprojection error")
        )
        self.declare_parameter('write_video', False,
            ParameterDescriptor(description="If set to True, it will create all output videos in the `/{video_dir}` directory")
        )
        self.declare_parameter('video_dir', 'videos',
           ParameterDescriptor(description="The name of the directory where video files are saved")
        )
        self.declare_parameter('video_fps', 30,
           ParameterDescriptor(description="The frames per second (FPS) of the output videos")
        )


        # Set the camera calibration information from config file
        try:
            (
             self.camera_mat,
             self.dist_coeffs,
             self.rep_error
            ) = self.read_camera_config_file()
        except Exception as e:
                self.get_logger().error(f"Could not read camera configuration file: {e}")
                exit(0)


        # Video writer configuration
        self.write_video = self.get_parameter('write_video').get_parameter_value().bool_value
        if self.write_video:
            video_dir = self.get_parameter('video_dir').get_parameter_value().string_value

            self.video_writer = None
            self.frame_size = None
            self.fps = (
                self.get_parameter('video_fps').get_parameter_value().integer_value
            )
            self.video_dir = "videos"

            os.makedirs(self.video_dir, exist_ok=True)
            self.video_path = os.path.join(video_dir, "output_video.mp4")

        # Aruco detector configuration
        self.ids_to_detect = [0] # TODO: Make this configurable
        self.dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # TODO: Make this configurable. By default, it's 4x4_50=0
        self.detector_params = aruco.DetectorParameters() # TODO: Look into this
        self.tracker = aruco.ArucoDetector(self.dict, self.detector_params)
        self.marker_length = 0.175 # meters TODO: configureable
        self.obj_points = np.array([ # Real world 3D coordinates of the marker corners
                [-self.marker_length / 2, 0,  self.marker_length / 2], # Top-left
                [ self.marker_length / 2, 0,  self.marker_length / 2], # Top-right
                [ self.marker_length / 2, 0, -self.marker_length / 2], # Bottom-right
                [-self.marker_length / 2, 0, -self.marker_length / 2], # Bottom-left
            ])

        # Subscriber configuration
        self.subscription = self.create_subscription(
                Image, 'image', self.image_callback, 1)
        self.bridge = CvBridge()

    def read_camera_config_file(self):
        """Read the camera configuration file and return parameters"""
        camera_config_file = self.get_parameter('camera_config_file').get_parameter_value().string_value
        fs = cv.FileStorage(camera_config_file, cv.FILE_STORAGE_READ)
        camera_mat = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        rep_error = fs.getNode("reproj_error").real()
        fs.release()

        return camera_mat, dist_coeffs, rep_error

    def image_callback(self, image_msg):
        try:
            # Convert ROS2 Image to OpenCv format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            if self. self.video_writer is None:
                self.frame_size = (cv_image.shape[1], cv_image.shape[0])
                self.video_writer = cv.VideoWriter(
                    self.video_path,
                    cv.VideoWriter_fourcc(*'mp4v'),
                    self.fps,
                    self.frame_size
                )
                self.get_logger().info(f"Video recording started: {self.video_path}")

            #self.get_logger().info("Image captured")

            # TODO: Grayscaling/Image Processing

            # Detect the marker ids
            marker_corners, marker_ids = self.detect_aruco_markers(cv_image)

            cv_detection_image = aruco.drawDetectedMarkers(cv_image, 
                                                           marker_corners,
                                                           marker_ids)

            # Calculate pose for each marker
            if (marker_ids is not None and len(marker_ids) != 0):
                for img_points in marker_corners:
                    img_points = np.array(img_points, dtype=np.float32).reshape(4, 2)
                    retval, rvec, tvec = cv.solvePnP(
                        self.obj_points, img_points, self.camera_mat, self.dist_coeffs
                    )
                    dist = np.linalg.norm(tvec)
                    self.get_logger().info(f"Distance to marker: {dist}")

            # Preview the images
            cv.waitKey(1)
            cv.imshow('untouched frame', cv_image)
            cv.imshow('tracked_markers', cv_detection_image)

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
