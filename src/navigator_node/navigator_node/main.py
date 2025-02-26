from dataclasses import dataclass
from enum import Enum

import rclpy
from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import PoseStamped
from loguru import logger as llogger
from rclpy.client import Client
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
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
    _rover_coord_subscription: Subscription
    """Subscribes to the rover's coordinate information."""
    _aruco_marker_pose_subscription: Subscription | None = None
    """Subscribes to the aruco's markers pose information."""
    _rover_imu_subscription: Subscription
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

    def __init__(self):
        """Create a new `NavigatorNode`"""
        self._coord_queue: list[GeoPoint] = []
        """The list of GNSS coordinates to traverse to."""

        super().__init__("navigator_node")

        # PARAMETERS

        # Get the rover's task (Navigating to GNSS corodinate, Navigating to ArUco marker, etc.)
        self.declare_parameter("rover_task", Parameter.Type.STRING)
        try:
            self._rover_task = RoverTask[self.get_parameter("rover_task").value]
        except KeyError:
            raise KeyError(
                f"The value of `rover_task`={self.get_parameter('rover_task').value} is not a valid rover task. Refer to `RoverTask` for all possible values."
            )
        llogger.info(
            f"Preparing Navigator Node for the following task: {self._rover_task}"
        )

        # If the rover's task is to not stand still, get the first coordinate to navigate to
        if self._rover_task != RoverTask.STAY_STILL_BRO:
            # Get the destination coordinate
            self.declare_parameter("coord_latitude", Parameter.Type.DOUBLE)
            self.declare_parameter("coord_longitude", Parameter.Type.DOUBLE)
            dest_coord = GeoPoint(
                latitude=self.get_parameter("coord_latitude").value,
                longitude=self.get_parameter("coord_longitude").value,
            )

            # Add it to the coordinate queue
            self._coord_queue.append(dest_coord)
            llogger.info(
                f"Set to navigate to: \n- ({dest_coord.latitude}, {dest_coord.longitude})"
            )

        # PUBLISHERS
        self._wheels_publisher = self.create_publisher(
            msg_type=WheelsMessage,
            topic="control/wheels",
            qos_profile=QUEUE_SIZE,
        )
        llogger.info("Finished making publishers!")

        # SUBSCRIBERS
        self._rover_coord_subscription = self.create_subscription(
            msg_type=GeoPointStamped,
            topic="/sensors/gps",
            callback=self.rover_coord_callback,
            qos_profile=QUEUE_SIZE,
        )

        self._rover_imu_subscription = self.create_subscription(
            msg_type=Imu,
            topic="/sensors/imu",
            callback=self.rover_imu_callback,
            qos_profile=QUEUE_SIZE,
        )

        # Only create ArUco marker pose subscription if we're tracking ArUco markers
        if self._rover_task == RoverTask.ARUCO_MARKER:
            self._aruco_marker_pose_subscription = self.create_subscription(
                msg_type=PoseStamped,
                topic="/aruco",  # TODO: change to whatever the markers topic is
                callback=self.aruco_marker_pose_callback,
                qos_profile=QUEUE_SIZE,
            )
        llogger.info("Finished making subscribers!")

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
        wheel_speeds_msg = WheelsMessage()
        match self._rover_task:
            case RoverTask.STAY_STILL_BRO:
                wheel_speeds_msg.left_wheels = 0
                wheel_speeds_msg.right_wheels = 0
                llogger.info("The rover is just a chill guy")

        self._wheels_publisher.publish(wheel_speeds_msg)

    def rover_coord_callback(self, msg: GeoPointStamped):
        """Callback to set rover coordinate information."""
        self._last_rover_coord = msg
        pass

    def aruco_marker_pose_callback(self, msg: PoseStamped):
        """Callback to set marker pose information."""
        self._last_marker_pose = msg
        pass

    def rover_imu_callback(self, msg: Imu):
        """Callback to set rover imu information."""
        self._last_rover_imu = msg
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
