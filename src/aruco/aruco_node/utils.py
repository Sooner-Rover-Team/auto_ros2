import cv2 as cv
import numpy as np
from cv2.typing import MatLike  # an object that behaves like a matrix
from scipy.spatial.transform import (
    Rotation as R,  # SciPy for quaternion conversion
)


def calc_object_pose(
    object_points: MatLike,
    image_points: MatLike,
    camera_matrix: MatLike,
    distance_coeffs: MatLike,
) -> tuple[bool, MatLike, list[MatLike]]:
    """Given the points of an object on a 2D image, return its 3D pose in the robot's coordinate frame."""
    success, rotation_vector, translation_vector = cv.solvePnP(
        object_points, image_points, camera_matrix, distance_coeffs
    )

    if not success:
        return False, np.zeros(4), np.zeros(3)  # pyright: ignore[reportReturnType]

    # Convert OpencCV's rotation vector to a rotation matrix
    rotation_matrix_cv, _ = cv.Rodrigues(rotation_vector)

    # A transformation matrix to convert from OpenCV's coordinate system (x: right, y: down, z: forward)
    # to the robot's coordinate system (z: right, x: up, y: forward)
    # OpenCV's Coordinate system
    cv_to_robot = np.array(
        [
            [0, 0, 1],
            [0, -1, 0],
            [1, 0, 0],
        ]
    )

    # Apply coordinate transformation to rotation matrix
    # i.e. from OpenCV's coordinate system to robot's coordinate system
    rotation_matrix_robot = rotation_matrix_cv @ cv_to_robot

    # Convert rotation matrix to quaternion
    orientation_quaternion = R.from_matrix(rotation_matrix_robot).as_quat()

    # Apply coordinate transformation to translation vector
    translation_robot = (cv_to_robot @ translation_vector).flatten()

    return success, orientation_quaternion, translation_robot  # pyright: ignore[reportReturnType]
