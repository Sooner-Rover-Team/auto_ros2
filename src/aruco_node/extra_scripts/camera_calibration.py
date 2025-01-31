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
        if (len(marker_corners) > 0):
            preview_frame = aruco.drawDetectedMarkers(preview_frame, marker_corners)

        if (charuco_corners is not None and len(charuco_corners) > 3):
            preview_frame = aruco.drawDetectedCornersCharuco(
                    preview_frame, charuco_corners, charuco_ids
                )

        # Preview the image
        cv2.putText(preview_frame, "Press 'c' to add current frame. 'q' to finish and calibrate",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2);
        cv2.imshow('calibration', preview_frame)

        # Check if user wants to calibrate with the frame or quit the program
        key = cv2.waitKey(1)
        if key == ord('q'):
            print('Quitting...')
            break


    # Release camera and destroy opencv windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
