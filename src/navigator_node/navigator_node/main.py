from enum import Enum
from dataclasses import dataclass, field
from loguru import logger as llogger

# rclpy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.timer import Timer

# ROS/Custom Message Types
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from custom_interfaces.msg import WheelsMessage

# how long we'll keep the data (DDS).
QUEUE_SIZE: int = 10


class RoverTask(Enum):
    """
    Describes the current rover task:
        - GNSS Coordinate
        - ArUco Marker
        - Object Detection
    """

    STAY_STILL_BRO = 0  # For debug purposes
    GNSS_COORD = 1
    ARUCO_MARKER = 2
    OBJECT_DETECTION = 3


class NavigationMode(Enum):
    """Describes the current navigation mode of the rover."""

    STANDBY = 0
    NAV_TO_GNSS = 1
    CALC_ARUCO_MARKER_POSE = 2


@dataclass(kw_only=True)
class NavigatorNode(Node):
    # ARGUMENTS
    _rover_task = RoverTask
    """The rover's current task."""

    # PUBLISHERS
    _wheels_publisher: Publisher
    """Publish wheel speeds to move the Rover."""

    # SUBSCRIBERS
    _gps_subscription: Subscription
    """Subscribes to the rover's coordinate information."""
    _aruco_subscription: Subscription
    """Subscribes to the aruco's markers pose information."""
    _imu_subscription: Subscription
    """Subscribes to the rover's imu information."""

    # MESSAGES
    _last_rover_coord: GeoPointStamped | None = None
    """The last known rover coordinate information."""
    _last_marker_pose: PoseStamped | None = None
    """The last known marker pose information."""
    _last_imu_data: Imu | None = None

    # SERVICE CLIENTS
    _lights_client: Client
    """Sends requests to change the rover's LED lights."""

    # TIMERS
    _navigator_callback_timer: Timer
    """
    Periodically call the navigation callback which sends wheel speeds
    """

    _aruco_marker_id: int | None = None
    """
    The ID of the ArUco marker we're looking for.
    (NOTE: This is only for logging as the `aruco_node` also has this as a parameter)
    """

    # STATE
    _times_marker_seen: int = 0
    """The amount of times that the ArUco marker has been seen."""

    _goal_reached: bool = False
    """Whether or not the rover task has been completed."""
    _coord_queue: list[GeoPoint] = field(default_factory=list)

    def __init__(self):
        """Create a new `NavigatorNode`"""
        super().__init__("navigator_node")

        # Get relevant parameters
        self.declare_parameter("rover_task", Parameter.Type.STRING)
        if not self.has_parameter("rover_task"):
            raise ValueError("Required parameter `rover_task` is missing")
        try:
            self._rover_task = RoverTask[self.get_parameter("rover_task").value]
        except KeyError:
            raise KeyError(
                f"The value of `rover_task`={self.get_parameter('rover_task').value} is not a valid rover task. Refer to `RoverTask` for all possible values."
            )
        llogger.info(
            f"Preparing Navigator Node for the following task: {self._rover_task}"
        )

        # Create publishers
        self._wheels_publisher = self.create_publisher(
            msg_type=WheelsMessage,
            topic="/controls/wheels",
            qos_profile=QUEUE_SIZE,
        )
        llogger.info("Finished making publishers!")

        # Create subscribers

        # Create services

        # Plot path to goal

        # Start navigating
        self._navigator_callback_timer = self.create_timer(0.5, self.navigator)
        llogger.info("Starting navigation!")

    def navigator(self):
        """Send wheel speeds depending on current mode of the rover."""
        llogger.debug("Navigating!")

        # Ensure navigation parameters were specified
        if self._rover_task is None:
            llogger.error("No rover task was specified!")
            self.destroy_node()
            rclpy.shutdown()

        # Check if goal was reached

        # Ensure we've started receiving rover coordinates

        # Ensure rover coordinates are updating

        # Send wheel speeds depending on our current mode
        wheel_speeds = WheelsMessage()
        match self._rover_task:
            case RoverTask.STAY_STILL_BRO:
                wheel_speeds.left_wheels = 0
                wheel_speeds.left_wheels = 0
                llogger.info("The rover is just a chill guy")

        self._wheels_publisher.publish(wheel_speeds)

    def rover_coord_callback(self, msg: GeoPointStamped):
        """Callback to set rover coordinate information."""
        pass

    def marker_pose_callback(self, msg: PoseStamped):
        """Callback to set marker pose information."""
        pass

    def rover_imu_callback(self, msg: Imu):
        """Callback to set rover imu information."""
        pass

    def __hash__(self) -> int:
        return super().__hash__()


def main(args: list[str] | None = None):
    """Spinup (start) the Navigator Node."""
    rclpy.init(args=args)
    navigator_node = NavigatorNode()

    try:
        rclpy.spin(navigator_node)
    except Exception as e:
        llogger.error(e)
        navigator_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
