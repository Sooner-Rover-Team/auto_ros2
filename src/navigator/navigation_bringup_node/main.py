import sys
from dataclasses import dataclass
from time import sleep

import rclpy
from geometry_msgs.msg import Twist
from loguru import logger as llogger
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node, Publisher, Subscription
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rclpy.timer import Timer
from typing_extensions import override

# the primary qos here is reliable - maps are sent reliably.
QOS_PROFILE: QoSProfile = QoSPresetProfiles.SERVICES_DEFAULT.value

MOVEMENT_SPEED: float = -0.2
"""How fast we'll move backward."""


@dataclass(kw_only=True)
class NavigationBringupNode(Node):
    _map_topic_subscriber: Subscription
    """Subscribes to the `/map` topic frame representation."""

    _wheels_publisher: Publisher
    """
    Publishes (very) slow speeds to the wheels for slight movement.

    This gives the `slam_toolbox` nodes enough information to compelte their
    `map -> odom -> base_link` chain!
    """

    _wheel_speeds_timer: Timer
    """A timer that says when to stop/start moving the wheels."""

    _is_done: bool = False
    """
    Indicates whether we've gotten a 'good' map.
    """

    def __init__(self):
        super().__init__("navigation_bringup_node")
        _ = self.get_logger().info("Performing navigation stack bringup...")

        # make a publisher to write to the wheels
        self._wheels_publisher = self.create_publisher(Twist, "/cmd_vel", QOS_PROFILE)
        _ = self.get_logger().debug("Created wheel publisher.")

        # on a timer, actually perform those publishes
        self._wheel_speeds_timer = self.create_timer(
            0.1,  # triggers every n seconds
            self._on_wheel_speeds_timer_tick,
        )
        _ = self.get_logger().debug("Made wheel publishing timer.")

        # wait for map to be not zero-sized
        self._map_topic_subscriber = self.create_subscription(
            OccupancyGrid, "/map", self._on_map_message, QOS_PROFILE
        )
        _ = self.get_logger().debug("Made map topic subscriber.")

    # all ROS 2 nodes must be hashable!
    @override
    def __hash__(self) -> int:
        return super().__hash__()

    def _on_map_message(self, map_message: OccupancyGrid):
        """
        Waits until the `slam_toolbox` publish a map that's not zero-sized.
        """

        # if the map has a size over 0x0...
        if (
            map_message.info.width > 0
            and map_message.info.height > 0
            and not self._is_done
        ):
            # stop this node!
            _ = self.get_logger().info(
                f"`/map` is now sized ({map_message.info.width}x{map_message.info.height})! Allowing dependent nodes to start..."
            )
            self.stop_wheels()
            self._wheel_speeds_timer.cancel()

            # say that we're 'done'
            self._is_done = True
            raise SystemExit()

    def _on_wheel_speeds_timer_tick(self):
        # decide which speeds to publish
        wheel_speed: Twist = Twist()
        wheel_speed.linear.x = MOVEMENT_SPEED

        # ...publish them.
        self._wheels_publisher.publish(wheel_speed)

    def stop_wheels(self):
        if rclpy.ok():
            # an empty message is 0 speed...
            _ = self.get_logger().info("Stopping wheels...")
            zero_speed_msg: Twist = Twist()

            # publish it to stop the wheels
            self._wheels_publisher.publish(zero_speed_msg)
            _ = self.get_logger().info("Wheels stopped.")


def main(args: list[str] | None = None):
    llogger.info("Starting Navigation bringup...")
    rclpy.init(args=args)
    node: NavigationBringupNode = NavigationBringupNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        llogger.info("Ctrl^C detected. Shutting down gracefully...")
    finally:
        # stop the wheels when we're done.
        #
        # then, turn off `rclpy` connection
        stop(node)


def stop(node: NavigationBringupNode):
    llogger.info("Calling `stop_wheels`...")
    node.stop_wheels()
    sleep(0.2)

    llogger.info("Destroying node. Goodbye!")
    node.destroy_node()

    rclpy.shutdown()
    llogger.info("Exiting...")
    sys.exit(0)


if __name__ == "__main__":
    main()
