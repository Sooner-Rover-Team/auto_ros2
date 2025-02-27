from dataclasses import dataclass
from enum import Enum

import rclpy
from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import PoseStamped
from geopy.distance import distance
from loguru import logger as llogger
from rclpy.client import Client
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from sensor_msgs.msg import Imu
from simple_pid import PID

from custom_interfaces.msg import WheelsMessage

from .coords import get_angle_to_dest

# how long we'll keep the data (DDS).
QUEUE_SIZE: int = 10

# Distances to complete goals for URC competition
MAX_DIST_FROM_GNSS_COORD = 3  # meters
MAX_DIST_FROM_ARUCO_MARKER = 2  # meters


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
    ROTATE_TO_GNSS = 2
    CALC_ARUCO_MARKER_POSE = 3


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
    _last_rover_imu_data: Imu | None = None
    """The last known rover imu information"""
    _last_rover_wheels_message: WheelsMessage | None = None
    """The last rover wheel speeds sent."""

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

    _nav_mode: NavigationMode = NavigationMode.STANDBY
    """The current mode of navigation."""

    # TODO: Make these parameters and not optional/default values
    _pid_kp: float
    """The value for the proportional gain in the PID controller"""

    _pid_ki: float
    """The value for the integral gain in the PID controller"""

    _pid_kd: float
    """The value for the derivative gain in the PID controller"""

    _pid_controller: PID
    """The PID controller used for any given task."""

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

            # Since we have a coordinate to navigate to, let's start by rotating to it
            self._nav_mode = NavigationMode.ROTATE_TO_GNSS

            # Create the pid controller needed to navigate to GNSS coordinates
            self._pid_pk, self._pid_pi, self._pid_pd = (
                0.04,
                0.3,
                0.05,
            )  # TODO: make these parameters again
            self._pid_controller = PID(
                self._pid_pk, self._pid_pi, self._pid_pd, setpoint=0
            )  # The goal for the PID controller is to make the rover's angle to the destination 0

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

        # Before we start navigating, make sure the rover is still
        wheel_speeds_msg = WheelsMessage(left_wheels=127, right_wheels=127)
        self._wheels_publisher.publish(wheel_speeds_msg)
        self._last_rover_wheels_message = wheel_speeds_msg
        # Start navigating
        self._navigator_callback_timer = self.create_timer(0.2, self.navigator)
        llogger.info("Starting navigation!")

    def navigator(self):
        """Send wheel speeds depending on current mode of the rover."""
        # Ensure navigation parameters were specified
        if self._rover_task is None:
            llogger.error("No rover task was specified!")
            self.destroy_node()
            rclpy.shutdown()

        # Ensure we've started receiving rover coordinates
        if self._last_rover_coord is None:
            llogger.info("Waiting for rover coordinates...")
            # NOTE: Maybe this should change nav mode?
            return

        # Check if goal was reached
        if self._goal_reached:
            llogger.info("The goal has been reached!")
            self._nav_mode = NavigationMode.STANDBY

        # Ensure rover coordinates are updating

        """
        0 = max reverse wheels
        126 = none
        255 = max speed forward
        """
        # Send wheel speeds depending on our current mode
        wheel_speeds_msg = WheelsMessage(left_wheels=0, right_wheels=0)
        dest_coord = self._coord_queue[0]
        match self._nav_mode:
            case NavigationMode.STANDBY:
                llogger.info("The rover is just a chill guy")
                wheel_speeds_msg.left_wheels = 127
                wheel_speeds_msg.right_wheels = 127
            case NavigationMode.NAV_TO_GNSS:
                llogger.info(
                    f"Rover is navigating to GNSS:\n- {dest_coord.latitude, dest_coord.longitude}"
                )
                # Calculate distance from current destination
                dist_from_coord = distance(
                    (
                        self._last_rover_coord.position.latitude,
                        self._last_rover_coord.position.longitude,
                    ),
                    (dest_coord.latitude, dest_coord.longitude),
                ).meters
                llogger.debug(f"Distance from coord: {dist_from_coord}")

                # See if we've made it to our destination
                # NOTE: May have to check if coordinate queue is empty too
                if dist_from_coord < MAX_DIST_FROM_GNSS_COORD:
                    # NOTE: Maybe reset pid controller?
                    self._goal_reached = True
                else:  # Else keep navigating
                    wheel_speeds_msg = self.wheel_speeds_for_nav_to_gnss(
                        dest_coord
                    )
            case NavigationMode.ROTATE_TO_GNSS:
                llogger.info(
                    f"Rover is rotating to GNSS:\n- {dest_coord.latitude, dest_coord.longitude}"
                )

                # If we're in line with the destination,
                # switch navigation mode to move to the GNSS coordinate
                # otherwise keep rotating.
                angle_error = get_angle_to_dest(
                    self._last_rover_coord, dest_coord, self._last_rover_imu
                )
                if angle_error < 10 and angle_error > -10:
                    self._nav_mode = NavigationMode.NAV_TO_GNSS
                else:
                    wheel_speeds_msg.left_wheels = 116
                    wheel_speeds_msg.right_wheels = 136

        llogger.debug(
            f"Sending the following wheel speeds:\n {wheel_speeds_msg}"
        )
        self._wheels_publisher.publish(wheel_speeds_msg)
        self._last_rover_wheels_message = wheel_speeds_msg

    def wheel_speeds_for_nav_to_gnss(self, dest_coord: GeoPoint):
        """Given a destination coordinate, give the wheel speeds needed reach the destination."""

        # Calculate the current PID value which based off of the rover's current angle to the destination.
        angle_error = get_angle_to_dest(
            self._last_rover_coord, dest_coord, self._last_rover_imu
        )
        llogger.debug(f"Angle to dest is {angle_error}")

        # If angle error is high, adjust the wheel speeds
        wheel_speeds_msg = WheelsMessage()
        if angle_error > 10 or angle_error < -10:
            control = self._pid_controller(angle_error)
            llogger.debug(f"Control value from PID is: {control}")
            if control is None:
                llogger.error("PID could not calculate correction value!")
                return

            wheel_speeds_msg.right_wheels = int(127 - control)
            wheel_speeds_msg.left_wheels = int(127 + control)
            # Make sure the wheel speeds are between 0-255
            wheel_speeds_msg.left_wheels = max(
                100, min(wheel_speeds_msg.left_wheels, 140)
            )
            wheel_speeds_msg.right_wheels = max(
                100, min(wheel_speeds_msg.right_wheels, 140)
            )
        else:  # Just go forward bro
            wheel_speeds_msg.right_wheels = 180
            wheel_speeds_msg.left_wheels = 180

        return wheel_speeds_msg

    def rover_coord_callback(self, msg: GeoPointStamped):
        """Callback to set rover coordinate information."""
        self._last_rover_coord = msg

    def aruco_marker_pose_callback(self, msg: PoseStamped):
        """Callback to set marker pose information."""
        self._last_marker_pose = msg

    def rover_imu_callback(self, msg: Imu):
        """Callback to set rover imu information."""
        self._last_rover_imu = msg

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
