import cv2 as cv
import numpy as np
import typer
from cv2.typing import MatLike  # something that behaves like a matrix
from typing_extensions import Annotated


def create_perfect_camera(image_width: int, image_height: int) -> list[MatLike]:
    """Create camera matrix and distance coefficients for a perfect camera (no distortions) with images that are 640x480."""
    # Calculate image center
    cx = image_width / 2
    cy = image_height / 2

    fx = fy = 800  # focal length (x and y) in pixels

    # The camera matrix explains the intrinsics of the camera (focal length, image center, etc.)
    camera_matrix: MatLike = np.array(
        [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64
    )

    # Distant coefficients tell use how the camera distorts an image
    # like by curving the image near the edges of the image
    dist_coeffs: MatLike = np.zeros((5, 1))

    return camera_matrix, dist_coeffs  # pyright: ignore[ reportReturnType]


def main(
    image_width: Annotated[
        int, typer.Argument(help="The width of the image with the aruco marker")
    ],
    image_height: Annotated[
        int,
        typer.Argument(help="The height of the image with the aruco marker"),
    ],
    marker_distance: Annotated[
        float,
        typer.Argument(
            help="How far (in meters) away the aruco marker is from the camera in the image"
        ),
    ],
    marker_rotation: Annotated[
        tuple[float, float, float],
        typer.Argument(
            help="The rotation of the marker (in degrees) relative to the camera (e.g. angle_x angle_y angle_z)"
        ),
    ],
    image_output: Annotated[
        str, typer.Argument(help="The path to the generated image")
    ],
):
    """
    Create a white image with an aruco marker placed a specific distance away (in meters) in the image.
    """
    # Create marker rotation and translation vectors describing the marker's
    # location relative to the camera.
    # Camera Coordinate System reference: https://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf#page=4
    marker_tvec = np.array(
        [
            0.0,
            0.0,
            marker_distance,
        ],
        dtype=np.float32,
    )

    # Create a rotation vector in Rodrigues form
    marker_rvec = np.array(marker_rotation, dtype=np.float32)  # Make into numpy array
    marker_rvec = cv.Rodrigues(np.deg2rad(marker_rvec))[
        0
    ]  # Convert rad to degrees and make Rodrigues form

    # Generate ArUco marker image (just to extract the pattern)
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    marker_img = cv.aruco.generateImageMarker(aruco_dict, 1, 100)

    # Calculate where the aruco marker corners are in relation to the center of the aruco marker in 3D space
    marker_length = 0.175  # correlates to URC standard marker length in meters
    half_size = marker_length / 2  # distance from center of aruco marker to perimeter
    object_points: MatLike = np.array(
        [
            [-half_size, half_size, 0],  # Top-left
            [half_size, half_size, 0],  # Top-right
            [half_size, -half_size, 0],  # Bottom-right
            [-half_size, -half_size, 0],  # Bottom-left
        ],
        np.float32,
    )

    # Assume we have a perfect camera (no distortions, rotation, etc.)
    camera_matrix, dist_coeffs = create_perfect_camera(image_width, image_height)

    # Create an empty white image
    image: MatLike = np.ones((480, 640, 3), dtype=np.uint8) * 255

    # Calculate where the image points for the object points are
    # (i.e. project the 3D points of the aruco marker onto our image)
    img_points, _ = cv.projectPoints(
        object_points,
        rvec=marker_rvec,
        tvec=marker_tvec,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
    )
    img_points = img_points.reshape(-1, 2).astype(int)

    # Warp the marker image to its projected quadrilateral on the blank image
    src_pts = np.array([[0, 99], [99, 99], [99, 0], [0, 0]], dtype=np.float32)
    dst_pts = img_points.astype(np.float32)
    M = cv.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv.warpPerspective(marker_img, M, (640, 480))

    # Create a mask and overlay the warped marker
    mask = cv.warpPerspective(np.ones_like(marker_img) * 255, M, (640, 480))
    mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
    warped = cv.cvtColor(warped, cv.COLOR_GRAY2BGR)

    # Blend into original image
    image = np.where(mask == 255, warped, image)

    _ = cv.imwrite(
        image_output,
        image,
    )


if __name__ == "__main__":
    typer.run(main)
