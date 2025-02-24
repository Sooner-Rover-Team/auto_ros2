from enum import Enum
from dataclasses import dataclass

# RCLPY Imports
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.timer import Timer

# ROS Message Types
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu  # TODO: Does this have to be custom?


class RoverTask(Enum):
    """Describes the current"""

    GNSS_COORD = 0
    ARUCO_MARKER = 1
    OBJECT_DETECTION = 2


class NavigationMode(Enum):
    """Describes the current navigation mode of the rover."""

    NAV_TO_GNSS = 0
    CALC_ARUCO_MARKER_POSE = 1


@dataclass(kw_only=True)
class NavigatorNode(Node):
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
    _coord_queue: list[GeoPoint] = []

    def __init__(self):
        """Create a new `NavigatorNode`"""
        super().__init__("Navigator Node")

        # Get relevant parameters
        pass
