#!/usr/bin/env python3
"""
Standalone ArUco Marker Visualizer

A simple Python script that subscribes to your camera feed and displays
detected ArUco markers with visual overlays.

No ROS package setup required - just run it!

Usage:
    python3 visualizer.py

Options:
    --image-topic IMAGE_TOPIC       Topic to subscribe to (default: /sensors/mono_image)
    --aruco-dict DICT_NAME          ArUco dictionary to use (default: 4x4_50)
"""

import argparse
import sys
from typing import Optional

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage


class ArucoVisualizer:
    """Simple standalone ArUco visualizer."""

    def __init__(self, image_topic: str, aruco_dict_name: str):
        """
        Initialize the visualizer.

        Args:
            image_topic: ROS topic to subscribe to
            aruco_dict_name: ArUco dictionary name (e.g., "4x4_50")
        """
        self.image_topic = image_topic
        self.bridge = CvBridge()
        self.current_image: Optional[np.ndarray] = None
        self.running = True

        # Initialize ArUco detector
        aruco_dict_map = {
            "4x4_50": aruco.DICT_4X4_50,
            "4x4_100": aruco.DICT_4X4_100,
            "4x4_250": aruco.DICT_4X4_250,
            "4x4_1000": aruco.DICT_4X4_1000,
            "5x5_50": aruco.DICT_5X5_50,
            "5x5_100": aruco.DICT_5X5_100,
            "5x5_250": aruco.DICT_5X5_250,
            "5x5_1000": aruco.DICT_5X5_1000,
            "6x6_50": aruco.DICT_6X6_50,
            "6x6_100": aruco.DICT_6X6_100,
            "6x6_250": aruco.DICT_6X6_250,
            "6x6_1000": aruco.DICT_6X6_1000,
            "7x7_50": aruco.DICT_7X7_50,
            "7x7_100": aruco.DICT_7X7_100,
            "7x7_250": aruco.DICT_7X7_250,
            "7x7_1000": aruco.DICT_7X7_1000,
        }

        if aruco_dict_name not in aruco_dict_map:
            raise ValueError(f"Unknown ArUco dictionary: {aruco_dict_name}")

        aruco_dict = aruco.getPredefinedDictionary(aruco_dict_map[aruco_dict_name])
        detector_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, detector_params)

        print(f"[INFO] Using ArUco dictionary: {aruco_dict_name}")
        print(f"[INFO] Subscribing to: {image_topic}")
        print("[INFO] Press 'q' in the window to quit\n")

    def image_callback(self, msg: RosImage):
        """Callback when a new image arrives."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            print(f"[ERROR] Failed to convert image: {e}")

    def draw_markers(self, frame: np.ndarray) -> tuple[np.ndarray, int]:
        """Detect and draw ArUco markers on the frame."""
        display_frame = frame.copy()

        # Detect markers
        marker_corners, marker_ids, _rejected = self.detector.detectMarkers(frame)

        num_markers = len(marker_ids) if marker_ids is not None else 0

        if marker_ids is not None and num_markers > 0:
            for i, marker_id in enumerate(marker_ids):
                corners = marker_corners[i][0].astype(int)

                # Draw bounding box (green)
                cv2.polylines(display_frame, [corners], True, (0, 255, 0), 2)

                # Draw corner circles (blue)
                for corner in corners:
                    cv2.circle(display_frame, tuple(corner), 5, (255, 0, 0), -1)

                # Draw ID at center
                center_x = int(np.mean(corners[:, 0]))
                center_y = int(np.mean(corners[:, 1]))

                id_text = f"ID: {int(marker_id[0])}"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                thickness = 2
                text_size = cv2.getTextSize(id_text, font, font_scale, thickness)[0]

                # Background for text
                pt1 = (
                    center_x - text_size[0] // 2 - 5,
                    center_y - text_size[1] // 2 - 5,
                )
                pt2 = (
                    center_x + text_size[0] // 2 + 5,
                    center_y + text_size[1] // 2 + 5,
                )
                overlay = display_frame.copy()
                cv2.rectangle(overlay, pt1, pt2, (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)

                # Draw text
                cv2.putText(
                    display_frame,
                    id_text,
                    (center_x - text_size[0] // 2, center_y + text_size[1] // 2),
                    font,
                    font_scale,
                    (0, 255, 0),
                    thickness,
                )

        # Draw stats
        stats_text = f"Detected: {num_markers} marker(s)"
        overlay = display_frame.copy()
        cv2.rectangle(overlay, (5, 5), (280, 40), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, display_frame, 0.7, 0, display_frame)
        cv2.putText(
            display_frame,
            stats_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        return display_frame, num_markers

    def run(self):
        """Main loop - subscribe and display."""
        rclpy.init()
        node = rclpy.create_node("aruco_visualizer")
        try:
            node.create_subscription(RosImage, self.image_topic, self.image_callback, 1)

            print("[INFO] Waiting for images...\n")

            while self.running:
                rclpy.spin_once(node, timeout_sec=0.01)

                if self.current_image is not None:
                    display_frame, num_markers = self.draw_markers(self.current_image)
                    cv2.imshow("ArUco Markers", display_frame)

                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        self.running = False
                else:
                    cv2.waitKey(1)

        except KeyboardInterrupt:
            print("\n[INFO] Interrupted")
        finally:
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()


def main():
    """Parse args and run."""
    parser = argparse.ArgumentParser(description="Standalone ArUco Marker Visualizer")

    parser.add_argument(
        "--image-topic",
        default="/sensors/mono_image",
        help="ROS topic for camera images (default: /sensors/mono_image)",
    )

    parser.add_argument(
        "--aruco-dict",
        default="4x4_50",
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
        help="ArUco dictionary (default: 4x4_50)",
    )

    args = parser.parse_args()

    try:
        visualizer = ArucoVisualizer(args.image_topic, args.aruco_dict)
        visualizer.run()
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
