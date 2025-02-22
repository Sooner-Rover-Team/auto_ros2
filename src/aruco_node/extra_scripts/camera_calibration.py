import cv2
import cv2.aruco as aruco
import argparse
import dataclasses
from loguru import logger
from typing import Sequence
from aruco_node.aruco_dict_map import aruco_dict_map


@dataclasses.dataclass
class Arguments:
    num_squares_x: int
    num_squares_y: int
    square_length: float
    marker_length: float
    camera_index: int
    output_file: str
    aruco_dict: aruco.Dictionary

    def __init__(self, args):
        self.num_squares_x = args.num_squares_x
        self.num_squares_y = args.num_squares_y
        self.square_length = args.square_length
        self.marker_length = args.marker_length
        self.camera_index = args.camera_index
        self.output_file = args.output_file
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_map[args.aruco_dict])


def add_arguments():
    """Parse the commandline arguements for the camera calibration script."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--num_squares_x",
        type=int,
        default=11,
        help="Number of columns in the aruco checkerboard",
    )
    parser.add_argument(
        "--num_squares_y",
        type=int,
        default=8,
        help="Number of rows in the aruco checkerboard",
    )
    parser.add_argument(
        "--square_length",
        type=float,
        default=0.02,
        help="Length of the checkerboard square that the marker lies in",
    )
    parser.add_argument(
        "--marker_length", type=float, default=0.015, help="Length of the marker itself"
    )
    parser.add_argument(
        "--camera_index", type=int, default=0, help="Index of the camera to use"
    )
    parser.add_argument(
        "--output_file",
        type=str,
        default="cam.yml",
        help="Output file to save the camera calibration info",
    )
    parser.add_argument(
        "--aruco_dict",
        type=str,
        choices=[
            "4x4_50",
            "4x4_100",
            "4x4_250",
            "4x4_1000",
            "5x5_50",
            "5x5_100",
            "5x5_250",
            "5x5_1000",
            "6x6_50",
            "6x6_100",
            "6x6_250",
            "6x6_1000",
            "7x7_50",
            "7x7_100",
            "7x7_250",
            "7x7_1000",
        ],
        default="5x5_50",
        help="Aruco dictionary to use. Takes argument of the form [# of bits]x[# of bits]_[# of aruco markers]. Default: 5x5_50",
    )

    # Return the arguments as a dataclass
    args = parser.parse_args()
    return Arguments(args)


# TODO: Divide this into smaller functions
def main():
    # Get aruco information from arguments
    args = add_arguments()

    # Create video capture device
    # TODO: Maybe allow a video file as well
    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        logger.error("Could not open camera")
        exit()

    # Create Charuco board and detector
    charuco_board = cv2.aruco.CharucoBoard(
        (args.num_squares_x, args.num_squares_y),
        args.square_length,
        args.marker_length,
        args.aruco_dict,
    )
    charuco_params = cv2.aruco.CharucoParameters()
    detector_params = cv2.aruco.DetectorParameters()
    charuco_detector = cv2.aruco.CharucoDetector(
        charuco_board, charuco_params, detector_params
    )

    # Prompt the user for calibration images to use for calibration
    captured_charuco_corners = []
    captured_charuco_ids = []
    captured_image_points: Sequence[cv2.typing.MatLike] = []
    captured_object_points: Sequence[cv2.typing.MatLike] = []
    captured_images = []
    image_size: cv2.typing.Size = []
    while cap.isOpened():
        # Grab the frame
        ret, frame = cap.read()
        preview_frame = frame.copy()
        if not ret:
            logger.info("Could not grab frame! Trying again...")
            continue

        # Try to detect the charuco board
        (charuco_corners, charuco_ids, marker_corners, marker_ids) = (
            charuco_detector.detectBoard(frame)
        )

        # Draw the detected corners and markers
        if charuco_corners is not None:
            preview_frame = aruco.drawDetectedCornersCharuco(
                preview_frame, charuco_corners, charuco_ids
            )

        # Preview the image
        cv2.putText(
            preview_frame,
            "Press 'c' to add current frame. 'q' to finish and calibrate",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )
        cv2.imshow("calibration", preview_frame)

        # Check if user wants to calibrate with the frame or quit the program
        key = cv2.waitKey(1)

        # Use current frame for calibration if:
        # - user presses 'c'
        # - more than 3 charuco markers were found
        if (
            key == ord("c")
            and charuco_corners is not None
            and marker_corners is not None
            and len(charuco_corners) > 3
        ):
            logger.debug(f"Markers found: {len(marker_ids)}")
            obj_points, img_points = charuco_board.matchImagePoints(
                charuco_corners, charuco_ids, obj_points=None, img_points=None
            )

            if obj_points is None or img_points is None:
                logger.info("Point matching failed, try again")
                continue

            logger.info("Frame captured")

            captured_charuco_corners.append(charuco_corners)
            captured_charuco_ids.append(charuco_ids)
            captured_image_points.append(img_points)
            captured_object_points.append(obj_points)
            captured_images.append(frame)
            image_size = frame.shape[0:2]

        if key == ord("q"):
            logger.info("Quitting...")
            break

    # Make sure at least 5 images are used for calibration
    if len(captured_charuco_corners) < 4:
        logger.info("Not enough corners for calibration")
        # Release camera and destroy opencv windows
        cap.release()
        cv2.destroyAllWindows()
        exit(0)

    # Calibrate the camera using ChAruco
    (rep_error, camera_mat, dist_coeffs, rvecs, tvecs) = cv2.calibrateCamera(
        captured_object_points, captured_image_points, image_size
    )

    # Save the camera parameters
    try:
        fs = cv2.FileStorage(args.output_file, cv2.FILE_STORAGE_WRITE)
        fs.write("camera_matrix", camera_mat)
        fs.write("dist_coeffs", dist_coeffs)
        fs.write("reproj_error", rep_error)
        fs.release()
    except Exception as e:
        logger.error(
            f"Could not finish writing calibration info to {args.output_file}: {e}"
        )
    finally:
        # Release camera and destroy opencv windows
        cap.release()
        cv2.destroyAllWindows()
        exit(0)

    logger.info(f"Rep Error: {rep_error}")
    logger.info(f"Calibration saved to {args.output_file}")

    # Release camera and destroy opencv windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
