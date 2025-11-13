import cv2 as cv
import numpy as np
from cv2.typing import MatLike  # an object that behaves like a matrix
from geometry_msgs.msg import Point, Pose, Quaternion
from numpy.typing import NDArray
from scipy.spatial.transform import (
    Rotation as R,  # SciPy for quaternion conversion
)


def calc_object_pose(
    object_points: MatLike,
    image_points: MatLike,
    camera_matrix: MatLike,
    distance_coeffs: MatLike,
) -> Pose | None:
    """
    Given the points of an object on a 2D image, return its 3D pose in the
    Rover's coordinate frame.

    This pose is in ROS 2's `Pose` format.
    """
    res: tuple[NDArray[np.float64], NDArray[np.float64]] | None = (
        _calc_object_pose_inner(
            object_points, image_points, camera_matrix, distance_coeffs
        )
    )

    # unwrap the return of that inner func
    if res is None:
        return None
    translation_robot: NDArray[np.float64] = res[0]
    orientation_quaternion: NDArray[np.float64] = res[1]

    # make a new pose; we'll return it in a sec
    pose: Pose = Pose()

    # add translation from rover
    pose.position = Point()
    pose.position.x = translation_robot[0]
    pose.position.y = translation_robot[1]
    pose.position.z = translation_robot[2]

    # add rotation from rover
    pose.orientation = Quaternion()
    pose.orientation.x = orientation_quaternion[0]
    pose.orientation.y = orientation_quaternion[1]
    pose.orientation.z = orientation_quaternion[2]
    pose.orientation.w = orientation_quaternion[3]

    return pose


def _calc_object_pose_inner(
    object_points: MatLike,
    image_points: MatLike,
    camera_matrix: MatLike,
    distance_coeffs: MatLike,
) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
    (success, rotation_vector, translation_vector) = cv.solvePnP(
        object_points, image_points, camera_matrix, distance_coeffs
    )

    if not success:
        return None

    # Convert OpencCV's rotation vector to a rotation matrix
    rotation_matrix_cv, _ = cv.Rodrigues(rotation_vector)

    # A transformation matrix to convert from OpenCV's coordinate system (x: right, y: down, z: forward)
    # to the robot's coordinate system (z: right, x: up, y: forward)
    # OpenCV's Coordinate system
    cv_to_robot: NDArray[np.float64] = np.array(
        [
            [0, 0, 1],
            [0, -1, 0],
            [1, 0, 0],
        ]
    )

    # Apply coordinate transformation to rotation matrix
    # i.e. from OpenCV's coordinate system to robot's coordinate system
    rotation_matrix_robot: NDArray[np.float64] = rotation_matrix_cv @ cv_to_robot

    # Convert rotation matrix to quaternion
    orientation_quaternion: NDArray[np.float64] = R.from_matrix(
        rotation_matrix_robot
    ).as_quat()

    # Apply coordinate transformation to translation vector
    translation_robot: NDArray[np.float64] = (
        cv_to_robot @ translation_vector
    ).flatten()

    return (orientation_quaternion, translation_robot)
