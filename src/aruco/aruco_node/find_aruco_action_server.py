import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import time

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
            execute_callback=self._send_aruco_feedback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback)
        
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
            feedback_msg.marker_ids = ArucoNode.found_marker_info.marker_ids
            feedback_msg.marker_poses = ArucoNode.found_marker_info.marker_poses
            feedback_msg.time_last_image_arrived = ArucoNode.found_marker_info.time_last_image_arrived

            self.get_logger.info(f"Sending feedback: {feedback_msg.marker_ids}, {feedback_msg.marker_poses}, {feedback_msg.time_last_image_arrived}", throttle_duration_sec=1.0)
            # Publish feedback to the client
            goal_handle.publish_feedback(feedback_msg)
            # Sleep for a short duration to avoid busy-waiting
            time.sleep(0.1)
        
        # Goal no longer active (canceled or shut down)
        self.get_logger().info("Goal no longer active. Find Aruco action server is shutting down.")
        result = FindArucoWithPose.Result()
        return result
    
def main(args=None):
    try:
        with rclpy.init(args=args):
            node = FindArucoActionServer()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()

