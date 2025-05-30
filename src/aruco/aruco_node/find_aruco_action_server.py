import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from custom_interfaces.action import FindArucoWithPose
from src.aruco.aruco_node.types import FoundMarkerInformation
from src.aruco.aruco_node.main import ArucoNode

class FindArucoActionServer(Node):
    """Action server for providing Aruco feedback to other nodes."""
    def __init__(self):
        super().__init__('find_aruco_action_server')
        self._action_server = ActionServer(
            self,
            FindArucoWithPose,
            'find_aruco_with_pose',
            self._send_aruco_feedback)
    
    def _send_aruco_feedback(self, goal_handle):
        """Any time the FoundMarkerInformation changes, send feedback to the client."""
        feedback_msg = FindArucoWithPose.Feedback()
        # Get the latest found marker information from the ArucoNode
        feedback_msg.found_marker_information = FoundMarkerInformation(
            marker_ids=ArucoNode.found_marker_information.marker_ids,
            marker_poses=ArucoNode.found_marker_information.marker_poses,
            time_last_image_arrived=ArucoNode.found_marker_information.time_last_image_arrived
        )

        self.get_logger().info("Feedback: {}".format(feedback_msg.found_marker_information))
        # Publish feedback to the client
        goal_handle.publish_feedback(feedback_msg)
        # Indicate that the goal is still active
        return TODO
    
def main(args=None):
    try:
        with rclpy.init(args=args):
            node = FindArucoActionServer()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()

