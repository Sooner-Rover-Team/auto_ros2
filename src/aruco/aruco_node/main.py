from collections.abc import Sequence
from dataclasses import dataclass

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import rclpy

# Used to convert OpenCV Mat type to ROS Image type
# NOTE: This may not be the most effective, we could turn the image into an AVIF string or even define a custom ROS data type.
import rclpy.subscription
from cv2.typing import MatLike
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from loguru import logger as llogger
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.publisher import Publisher
from scipy.spatial.transform import (
    Rotation as R,  # SciPy for quaternion conversion
)
from sensor_msgs.msg import Image as RosImage
from typing_extensions import override

from .aruco_dict_map import aruco_dict_map


@dataclass(kw_only=True)
class ArucoNode(Node):
    """
    When <TODO: started or action start msg recv'd?>, this node will begin
    searching for ArUco markers in the environment. <TODO: see above> will
    provide a marker ID to find.

    If any marker is found, it sends feedback displaying which markers are
    currently in frame.

    <b>
    This <TODO: action/node> runs until stopped by the Navigator node.
    <TODO: how?> It's up to the caller to stop this node when we're close
    enough to the goal.

    In other words, the ArUco Node isn't making the calls - the Navigator
    manages us!
    </b>
    """

    camera_config_file: str
    """Filename of the config file."""

    aruco_dict_map: dict[str, int]
    """The set of marker's we're looking for"""

    aruco_dict: str
    """Name of that set."""

    marker_length: float
    """Size of marker in meters."""

    marker_id: int
    """The ArUco marker ID to look for."""

    aruco_detector: aruco.ArucoDetector
    """Detects aruco markers."""

    tracker: aruco.ArucoDetector
    """TODO: how is this different from the detector? same class lol"""

    detector_params: aruco.DetectorParameters
    """Used to change the defaults in the detector's initializer."""

    image_subscription: rclpy.subscription.Subscription
    """Image subscriber for aruco."""

    marker_pose_publisher: Publisher
    """Publishes marker poses for other nodes."""

    bridge: CvBridge = CvBridge()
    """Converts images from ROS to cv2.Mat types"""

    obj_points: MatLike

    # camera configuration variables
    camera_mat: MatLike
    dist_coeffs: MatLike
    rep_error: float

    def __init__(self):
        super().__init__("aruco_node")
        # Declare the parameters for the node
        self.camera_config_file = (
            self.declare_parameter(
                "camera_config_file",
                "cam.yml",
                ParameterDescriptor(
                    description="The camera config .yml file that contains camera matrix, distance coefficients, and reprojection error",
                    read_only=True,
                ),
            )
            .get_parameter_value()
            .string_value
        )
        self.aruco_dict = (
            self.declare_parameter(
                "aruco_dict",
                "4x4_50",  # correlates to URC standard dictionary
                ParameterDescriptor(
                    description="This is the aruco dictionary number to assume aruco tags are from which come from 'cv2.aruco' predefined dictionary enum",
                ),
            )
            .get_parameter_value()
            .string_value
        )
        self.marker_length = (
            self.declare_parameter(
                "marker_length",
                0.175,  # correlates to URC standard marker length
                ParameterDescriptor(
                    description="This is the length (in meters) of the one side of the aruco marker you are tring to detect (not including white border).",
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.marker_id = (
            self.declare_parameter(
                "marker_id",
                0,
                ParameterDescriptor(
                    description="The ArUco marker id to track.",
                ),
            )
            .get_parameter_value()
            .integer_value
        )

        # Camera calibration configuration
        (self.camera_mat, self.dist_coeffs, self.rep_error) = (
            self.read_camera_config_file()
        )
        _ = self.get_logger().debug(
            "Finished reading calibration information for camera"
        )

        # Aruco detector configuration
        self.detector_params = aruco.DetectorParameters()  # TODO: Look into this
        self.tracker = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco_dict_map[self.aruco_dict]),
            self.detector_params,
        )
        self.obj_points = np.array(
            [  # Real world 3D coordinates of the marker corners
                [
                    -self.marker_length / 2,
                    0,
                    self.marker_length / 2,
                ],  # Top-left
                [
                    self.marker_length / 2,
                    0,
                    self.marker_length / 2,
                ],  # Top-right
                [
                    self.marker_length / 2,
                    0,
                    -self.marker_length / 2,
                ],  # Bottom-right
                [
                    -self.marker_length / 2,
                    0,
                    -self.marker_length / 2,
                ],  # Bottom-left
            ]
        )
        llogger.debug("Finished creating aruco tracker")

        # Subscriber configuration
        # TODO: Should we crash if we can't connect to image topic?
        # self.latest_image = None # Most recent video capture frame from subscriber
        self.image_subscription = self.create_subscription(
            RosImage, "/sensors/mono_image", self.image_callback, 1
        )
        llogger.debug("Finished creating image subscriber")

        self.marker_pose_publisher = self.create_publisher(
            PoseStamped,
            "marker_pose",
            0,
        )
        llogger.debug("Finished creating image subscriber")

    def image_callback(self, image: RosImage):
        """Get an image from the image topic and look for aruco tags."""

        # Convert ROS image to cv2.Mat
        cv_image: cv.Mat = self.bridge.imgmsg_to_cv2(image)  # pyright: ignore[reportAssignmentType]

        # Detect the marker ids
        detected_marker_corners, detected_marker_ids = self.detect_aruco_markers(
            cv_image
        )

        # If we found the marker we're looking for,
        # calculate and publish its pose.
        if detected_marker_ids is not None:
            try:
                marker_id_index = list(detected_marker_ids).index(self.marker_id)

                # Calculate the markers pose
                calculated_pose, quaternion, tvec = self.calculate_pose(
                    detected_marker_corners[marker_id_index]
                )
                llogger.debug(calculated_pose)

                # Publish the markers transform
                if calculated_pose:
                    marker_pose_msg = PoseStamped()
                    marker_pose_msg.header.stamp = self.get_clock().now().to_msg()
                    # TODO: This should probably be a parameter
                    marker_pose_msg.header.frame_id = "camera"

                    marker_pose_msg.pose.position.x = tvec[0]
                    marker_pose_msg.pose.position.y = tvec[1]
                    marker_pose_msg.pose.position.z = tvec[2]

                    marker_pose_msg.pose.orientation.w = quaternion[0]
                    marker_pose_msg.pose.orientation.x = quaternion[1]
                    marker_pose_msg.pose.orientation.y = quaternion[2]
                    marker_pose_msg.pose.orientation.z = quaternion[3]

                    self.marker_pose_publisher.publish(marker_pose_msg)
                    llogger.debug(
                        f"Publishing pose of marker:\n {tvec[0]}, {tvec[0]}, {tvec[0]}"
                    )
                else:
                    llogger.error("Could not calculate pose of detected marker")
            except ValueError:
                # sometimes, we find other markers, but we don't really care
                # about those markers' transforms.
                #
                # so... we're done.
                return

    def detect_aruco_markers(
        self, image: cv.Mat
    ) -> tuple[Sequence[cv.typing.MatLike], cv.typing.MatLike | None]:
        """
        Given an image, return detected aruco markers and rejected markers
        (candidates for aruco markers)
        """
        # Detect markers (corners and ids) and possible corners (rejected)
        (detected_marker_corners, detected_marker_ids, _rejected_marker_ids) = (
            self.tracker.detectMarkers(image)
        )
        return detected_marker_corners, detected_marker_ids

    def calculate_pose(
        self, img_points: MatLike
    ) -> tuple[bool, MatLike, list[MatLike]]:
        """Given a set of image points (detected aruco corners), return the pose for the marker"""
        img_points = np.array(img_points, dtype=np.float32).reshape(4, 2)
        retval, cv_rvec, cv_tvec = cv.solvePnP(
            self.obj_points, img_points, self.camera_mat, self.dist_coeffs
        )

        # Convert from cv2 camera coordinate system to robotic coordinate system
        # (cv) x, y, z =>  (robot) z, x, y
        cam_tvec = [cv_tvec[2][0], cv_tvec[0][0], cv_tvec[1][0]]

        # Convert axis-angle format to rotation matrix format
        cv_rmatrix, __ = cv.Rodrigues(cv_rvec)
        cam_rmatrix: MatLike = np.matmul(  # pyright: ignore[reportAny]
            cv_rmatrix,
            np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]]),
        )
        cam_quaternion = R.from_matrix(cam_rmatrix).as_quat()

        return retval, cam_quaternion, cam_tvec

    def read_camera_config_file(self):
        """Read the camera configuration file and return parameters"""

        # Try to open camera config file
        fs = cv.FileStorage(self.camera_config_file, cv.FILE_STORAGE_READ)
        if not fs.isOpened():
            llogger.error(
                f"Error opening camera config file at {self.camera_config_file}"
            )
            exit(1)

        camera_mat = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        rep_error = fs.getNode("reproj_error").real()
        fs.release()

        return camera_mat, dist_coeffs, rep_error

    @override
    def __hash__(self) -> int:
        return super().__hash__()


def main(args: list[str] | None = None):
    """Handle spinning up and destroying a node"""
    rclpy.init(args=args)
    aruco_node = ArucoNode()

    try:
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        aruco_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
