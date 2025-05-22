import os
from scipy.spatial.transform import Rotation as R
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import pytest
from cv2.typing import MatLike
from pytest import approx

from aruco_node.utils import (
    calc_object_pose,
)


@pytest.fixture
def sim_aruco_dist_and_images() -> list[tuple[float, MatLike]]:
    """A list of simulated aruco images placed at varying distances from the camera."""
    directory_path = "tests/sim_dist_aruco_images"
    image_filenames = os.listdir(directory_path)

    dist_and_images: list[tuple[float, MatLike]] = []
    for image_filename in image_filenames:
        # Get the meter distance from the filename
        dist = float(image_filename.split("dist_")[1].split("m")[0].replace("_", "."))

        # Read the image
        image = cv.imread(f"{directory_path}/{image_filename}")
        dist_and_images.append((dist, image))

    # Return the distance that the aruco marker is from the image and the image
    # sorted by distance
    return sorted(dist_and_images, key=lambda dist_image: dist_image[0])


@pytest.fixture
def sim_aruco_rot_and_images() -> list[tuple[tuple[float, float, float], MatLike]]:
    """A list of simulated aruco images rotated relative to the camera in different ways."""
    directory_path = "tests/sim_rot_aruco_images"
    image_filenames = os.listdir(directory_path)

    rot_and_images = []
    for image_filename in image_filenames:
        # Get the rotations from the filename
        angles = image_filename.split("rot_")[1].split(".")[0].split("_")

        # Convert from strings to floats
        angles = (float(angles[0]), float(angles[1]), float(angles[2]))

        # Read the image
        image = cv.imread(f"{directory_path}/{image_filename}")
        rot_and_image = (angles, image)
        rot_and_images.append(rot_and_image)

    # Return the distance that the aruco marker is from the image and the image
    # sorted by distance
    return sorted(rot_and_images, key=lambda dist_image: dist_image[0])  # pyright: ignore[ reportReturnType]


@pytest.fixture
def aruco_object_points() -> MatLike:
    # Calculate where the aruco marker corners are in relation to the center of the aruco marker in 3D space
    marker_length = 0.175  # correlates to URC standard marker length in meters
    half_size = marker_length / 2  # distance from center of aruco marker to perimeter

    aruco_object_points: MatLike = np.array(
        [
            [-half_size, half_size, 0],  # Top-left
            [half_size, half_size, 0],  # Top-right
            [half_size, -half_size, 0],  # Bottom-right
            [-half_size, -half_size, 0],  # Bottom-left
        ],
        np.float32,
    )
    return aruco_object_points


@pytest.fixture
def perfect_camera() -> tuple[MatLike, MatLike]:
    """For simulated images, a perfect camera (no distortions) with images that are 640x480."""
    # Calculate image center
    cx = 640 / 2
    cy = 480 / 2

    fx = fy = 800  # focal length (x and y) in pixels

    # The camera matrix explains the intrinsics of the camera (focal length, image center, etc.)
    camera_matrix: MatLike = np.array(
        [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64
    )

    # Distant coefficients tell use how the camera distorts an image
    # like by curving the image near the edges of the image
    dist_coeffs: MatLike = np.zeros((5, 1))

    return camera_matrix, dist_coeffs


@pytest.fixture
def aruco_detector() -> aruco.ArucoDetector:
    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
        aruco.DetectorParameters(),
    )
    return detector


def test_aruco_dist_from_sim_image(
    sim_aruco_dist_and_images: list[tuple[list[float], MatLike]],
    aruco_object_points: MatLike,
    perfect_camera: tuple[MatLike, MatLike],
    aruco_detector: aruco.ArucoDetector,
):
    # Calculate the pose for each image and compare it to the known distance
    camera_matrix, dist_coeffs = perfect_camera
    for expected_dist, image in sim_aruco_dist_and_images:
        # Detect the markers (and their corners) in the image
        (
            detected_marker_corners,
            _,
            _,
        ) = aruco_detector.detectMarkers(image)

        # Calculate the distance from the camera to the marker
        success, quat, tvec = calc_object_pose(
            aruco_object_points,
            detected_marker_corners[0],
            camera_matrix,
            dist_coeffs,
        )
        approx_dist = np.linalg.norm(tvec)

        # Make sure the pose was calculated without error and accurately
        expected_tvec = np.array([expected_dist, 0, 0])
        expected_quat = np.array([0, 0, 0, 1])

        assert success
        assert np.allclose(quat, expected_quat, atol=1.0)
        assert np.allclose(tvec, expected_tvec, atol=1.0)
        assert expected_dist == approx(approx_dist, abs=1.0)


def test_aruco_rot_from_sim_image(
    sim_aruco_rot_and_images: list[tuple[float, MatLike]],
    aruco_object_points: MatLike,
    perfect_camera: tuple[MatLike, MatLike],
    aruco_detector: aruco.ArucoDetector,
):
    # Calculate the pose for each image and compare it to the known distance
    # Here, we're testing if rotation affects our distance calculation
    camera_matrix, dist_coeffs = perfect_camera
    for expected_rot, image in sim_aruco_rot_and_images:
        # Detect the markers (and their corners) in the image
        (detected_marker_corners, _, _) = aruco_detector.detectMarkers(image)

        # Calculate the distance from the camera to the marker
        success, quat, tvec = calc_object_pose(
            aruco_object_points,
            detected_marker_corners[0],
            camera_matrix,
            dist_coeffs,
        )
        approx_dist = np.linalg.norm(tvec)

        # Make sure the pose was calculated without error and accurately
        expected_tvec = [1.0, 0.0, 0.0]
        expected_dist = 1.0

        rot_matrix = R.from_euler("xyz", np.radians(expected_rot))
        expected_quat = rot_matrix.as_quat()

        assert success
        assert np.allclose(quat, expected_quat, atol=1.0)
        assert np.allclose(tvec, expected_tvec, atol=1.0)
        assert expected_dist == approx(approx_dist, abs=1.0)
