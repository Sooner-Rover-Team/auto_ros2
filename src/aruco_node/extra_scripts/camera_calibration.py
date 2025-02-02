import numpy as np
import cv2
import cv2.aruco as aruco

def main():
    # TODO: Make these command line arguments
    num_squares_x = 11
    num_squares_y = 8
    square_length = 0.02  # meters
    marker_length = 0.015 # meters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    camera_index = 0
    output_file = "cam.yml"
    # TODO: Add aspect_ratio. Maybe this will make the calibration better?

    # Create video capture device
    # TODO: Maybe allow a video file as well
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Could not open camera")
        exit()

    # Create Charuco board and detector
    charuco_board = cv2.aruco.CharucoBoard(
            (num_squares_x, num_squares_y), square_length, marker_length, aruco_dict
    )
    charuco_params = cv2.aruco.CharucoParameters()      # TODO: Allow the user to adjust 
    detector_params = cv2.aruco.DetectorParameters()    # TODO: Allow the user to adjust 
    charuco_detector = cv2.aruco.CharucoDetector(charuco_board, charuco_params, detector_params)

    # Prompt the user for calibration images to use for calibration
    captured_charuco_corners = []
    captured_charuco_ids = []
    captured_image_points = []
    captured_object_points = []
    captured_images = []
    image_size = None
    while (cap.isOpened()):
        # Grab the frame
        ret, frame = cap.read()
        preview_frame = frame.copy()
        if not ret:
            print("Could not grab frame! Trying again...")
            continue

        # Try to detect the charuco board
        (
         charuco_corners,
         charuco_ids,
         marker_corners,
         marker_ids
        ) = charuco_detector.detectBoard(frame)

        # Draw the detected corners and markers
        if (charuco_corners is not None and len(charuco_corners)):
            preview_frame = aruco.drawDetectedCornersCharuco(
                preview_frame, charuco_corners, charuco_ids)

        # Preview the image
        cv2.putText(preview_frame, "Press 'c' to add current frame. 'q' to finish and calibrate",
              (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2);
        cv2.imshow('calibration', preview_frame)

        # Check if user wants to calibrate with the frame or quit the program
        key = cv2.waitKey(1)

        # Use current frame for calibration if:
        # - user presses 'c'
        # - more than 3 charuco markers were found
        if (key == ord('c') and 
                charuco_corners is not None and
                marker_corners is not None and
                len(charuco_corners)  > 3):

            print(len(marker_corners))
            print(len(marker_ids))
            obj_points, img_points = charuco_board.matchImagePoints(
                    charuco_corners, charuco_ids)

            if (obj_points is None or img_points is None):
                print("Point matching failed, try again")
                continue

            print("Frame captured")


            captured_charuco_corners.append(charuco_corners)
            captured_charuco_ids.append(charuco_ids)
            captured_image_points.append(img_points)
            captured_object_points.append(obj_points)
            captured_images.append(frame)
            image_size = frame.shape[0:2]

        if key == ord('q'):
            print('Quitting...')
            break

    # Make sure at least 5 images are used for calibration
    if (len(captured_charuco_corners) < 4):
        print("Not enough corners for calibration")
        # Release camera and destroy opencv windows
        cap.release()
        cv2.destroyAllWindows()
        exit(0)

    # Calibrate the camera using ChAruco
    (
     rep_error, camera_mat, dist_coeffs, rvecs, tvecs
    )= cv2.calibrateCamera(
            captured_object_points, captured_image_points, image_size,
            cameraMatrix=None, distCoeffs=None
    )

    # Save the camera parameters
    try:
        fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
        fs.write("camera_matrix", camera_mat)
        fs.write("dist_coeffs", dist_coeffs)
        fs.write("reproj_error", rep_error)
        fs.release()
    except Exception as e:
        print(f"Could not finish writing calibration info to {output_file}: {e}")
    finally:
        # Release camera and destroy opencv windows
        cap.release()
        cv2.destroyAllWindows()
        exit(0)

    
    print(f"Rep Error: {rep_error}")
    print(f"Calibration saved to {output_file}")


    # Release camera and destroy opencv windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
