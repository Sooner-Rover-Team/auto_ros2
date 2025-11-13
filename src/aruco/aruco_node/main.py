from collections.abc import Sequence
from dataclasses import dataclass
from threading import Event, Lock

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import rclpy
import rclpy.subscription
from builtin_interfaces.msg import Time
from custom_interfaces.action import FindArucoWithPose
from cv2.typing import MatLike
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from loguru import logger as llogger
from numpy.typing import NDArray
from rcl_interfaces.msg import ParameterDescriptor

# Used to convert OpenCV Mat type to ROS Image type
# NOTE: This may not be the most effective, we could turn the image into an AVIF string or even define a custom ROS data type.
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.publisher import Publisher
from scipy.spatial.transform import (
    Rotation as R,  # SciPy for quaternion conversion
)
from sensor_msgs.msg import Image as RosImage
from src.aruco.aruco_node.types import FoundMarkerInformation
from typing_extensions import override

from .aruco_dict_map import aruco_dict_map
from .utils import calc_object_pose


@dataclass(kw_only=True)
class ArucoNode(Node):
    """
    When its `FindArucoWithPose` action server is started up, this node checks
    for ArUco markers via a constant scan.

    It reports back to the client with feedback showing which markers are
    visible, and at which positions they are at.
    """

    _found_marker_info: FoundMarkerInformation = FoundMarkerInformation()
    """Information about what markers we've found."""

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

    _action_server: rclpy.action.ActionServer | None = None

    # these two help sync the `_send_aruco_feedback` method with
    # `_mono_image_callback`.
    #
    # WARNING: these are NOT abstracted! be very careful to use the lock!!!
    _image_event: Event = Event()
    """Emitted when we get a new image from the camera."""
    _info_mutex: Lock = Lock()
    """A MutEx for `info` to avoid data races."""

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
                    description="The camera config (YAML file) that contains camera matrix, distance coefficients, and reprojection error",
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
        _ = self.get_logger().debug("Read calibration information for camera.")

        # ArUco detector configuration
        #
        # TODO: look into this
        self.detector_params = aruco.DetectorParameters()
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
        llogger.debug("Finished creating ArUco tracker")

        # Subscriber configuration
        #
        # TODO: Should we crash if we can't connect to image topic?
        self.image_subscription = self.create_subscription(
            RosImage, "/sensors/mono_image", self._mono_image_callback, 1
        )
        llogger.debug("Finished creating image subscriber")

        self.marker_pose_publisher = self.create_publisher(
            PoseStamped,
            "marker_pose",
            0,
        )
        llogger.debug("Finished creating image subscriber")

        # Set up the action server
        self._action_server = ActionServer(
            self,
            FindArucoWithPose,
            "find_aruco_with_pose",
            execute_callback=self._send_aruco_feedback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        llogger.debug("Finished creating action server")

    def _mono_image_callback(self, image: RosImage):
        """
        This function runs when we get a new image from `/sensors/mono_image`,
        which is our (primary) See3CAM color camera!

        It does the following:

        1. Pipes image into an OpenCV `Mat` for further processing
        2. Processes the `Mat` to check for ArUco markers visible in the image
        3. Finds the "pose" of each marker in the image
        4. Transforms each pose to use a Rover-local coordinate system
        5. Stores the list of ArUco marker IDs and their poses in the class
        6. Sends a signal to the action server(s) to ask them to update their
           clients with the new information.
        """
        # save the time we recv'd the image
        recv_time: Time = self.get_clock().now().to_msg()

        # convert the ROS 2 `Image` into an OpenCV `Mat`
        cv_image: cv.Mat = self.bridge.imgmsg_to_cv2(image)  # pyright: ignore[reportAssignmentType]

        # check for markers visible in the image
        dect_res = self.detect_aruco_markers(cv_image)
        marker_corners: Sequence[MatLike] = dect_res[0]
        optional_marker_ids: MatLike | None = dect_res[1]

        # if we didn't find anything, do an early return
        if optional_marker_ids is None:
            # remove marker stuff from message and update the image update time
            with self._info_mutex:
                self._found_marker_info.marker_ids = None
                self._found_marker_info.marker_poses = None
                self._found_marker_info.time_last_image_arrived = recv_time

            # signal that we got a new frame!
            self._image_event.set()

            # then return
            return

        # alright, so, we did find markers.
        #
        # let's calculate the Rover-local pose for each...
        marker_ids: list[int] = []
        marker_poses: list[Pose] = []
        for i in range(0, len(marker_corners)):
            # grab the corners + id for this marker
            corners: MatLike = marker_corners[i]
            id: int = int(optional_marker_ids[i][0])

            # grab its pose
            pose: Pose | None = calc_object_pose(
                self.obj_points, corners, self.camera_mat, self.dist_coeffs
            )

            # only add it when it's not `None`
            if pose is not None:
                marker_ids.append(id)
                marker_poses.append(pose)

        # set our marker info list
        with self._info_mutex:
            self._found_marker_info.marker_ids = marker_ids
            self._found_marker_info.marker_poses = marker_poses
            self._found_marker_info.time_last_image_arrived = recv_time

        # signal new frame
        self._image_event.set()

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
    ) -> tuple[MatLike, list[MatLike]] | None:
        """
        Given a set of points in an image (which will be corners of detected
        ArUco markers), this method returns their poses in 3D space.

        Poses are converted to be in the Rover's coordinate format.
        """

        # numpy it up
        np_points: NDArray[np.float32] = np.array(
            img_points, dtype=np.float32
        ).reshape(4, 2)

        # stick needed info into OpenCV's `solve_pnp` for info about where the
        # ArUco markers are in 3D space.
        found_any_markers, cv_rvec, cv_tvec = cv.solvePnP(
            self.obj_points, np_points, self.camera_mat, self.dist_coeffs
        )

        # note that `found_any_markers` (the first thing in the tuple above)
        # represents whether we got anything useful. when it's `false`, we'll
        # just return `None` to the caller
        if not found_any_markers:
            return None

        # Convert from cv2 camera coordinate system to robotic coordinate system
        # (cv) x, y, z =>  (robot) z, x, y
        cam_tvec: list[MatLike] = [cv_tvec[2][0], cv_tvec[0][0], cv_tvec[1][0]]

        # Convert axis-angle format to rotation matrix format
        cv_rmatrix, __ = cv.Rodrigues(cv_rvec)
        cam_rmatrix: MatLike = np.matmul(  # pyright: ignore[reportAny]
            cv_rmatrix,
            np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]]),
        )
        cam_quaternion: MatLike = R.from_matrix(cam_rmatrix).as_quat()

        return (cam_quaternion, cam_tvec)

    def read_camera_config_file(self) -> tuple[MatLike, MatLike, float]:
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

    def _goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        self.get_logger().info("Received a new goal request.")
        # Accept the goal request
        return rclpy.action.GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info("Received a cancel request.")
        # Send cancel response
        return rclpy.action.CancelResponse.ACCEPT

    def _send_aruco_feedback(self, goal_handle):
        """Any time the FoundMarkerInformation changes, send feedback to the client."""
        while rclpy.ok() and goal_handle.is_active():
            feedback_msg = FindArucoWithPose.Feedback()

            # Get the latest found marker information from the ArucoNode
            MSG_TIMEOUT: float = 2.0
            if self._image_event.wait(timeout=MSG_TIMEOUT):
                with self._info_mutex:
                    feedback_msg.marker_ids = self._found_marker_info.marker_ids
                    feedback_msg.marker_poses = (
                        self._found_marker_info.marker_poses
                    )
                    feedback_msg.time_last_image_arrived = (
                        self._found_marker_info.time_last_image_arrived
                    )
            else:
                self.get_logger().warn(
                    f"It's been {MSG_TIMEOUT} seconds, but no new image found!"
                )
                continue
            pass

            self.get_logger().info(
                f"Sending feedback: {feedback_msg.marker_ids}, {feedback_msg.marker_poses}, {feedback_msg.time_last_image_arrived}",
                throttle_duration_sec=1.0,
            )

            # Publish feedback to the client
            goal_handle.publish_feedback(feedback_msg)

            # clear the event flag
            self._image_event.clear()
        pass

        # Goal no longer active (canceled or shut down)
        self.get_logger().info(
            "Goal no longer active. Find Aruco action server is shutting down."
        )
        result = FindArucoWithPose.Result()
        return result

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
