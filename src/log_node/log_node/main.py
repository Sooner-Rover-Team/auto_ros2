from dataclasses import dataclass

import rclpy
from loguru import logger
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from rpyutils.add_dll_directories import sys
from typing_extensions import override

## how long we'll keep the data (DDS).
QUEUE_SIZE: int = 10

## config for the qos, making it volatile like `rosout`
QOS_PROFILE = QoSProfile(
    depth=QUEUE_SIZE,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(kw_only=True)
class LogNode(Node):
    """A node that logs everything from `rosout` onto the terminal."""

    ## to make us a subscriber
    _subscription: Subscription

    def __init__(self):
        """
        Creates a new `Self`.
        """

        # give the node a name
        super().__init__("log_node")

        # subscribe to the topic
        self._subscription = self.create_subscription(
            msg_type=Log,
            topic="/rosout",
            callback=self._msg_callback,
            qos_profile=QOS_PROFILE,
        )

    def _msg_callback(self, log: Log):
        """
        This function runs each time a log message runs through `rosout`.
        """
        # grab the log's message
        msg: str = f"[{log.name}] {log.msg}"

        # print it out with loguru
        match log.level:
            case Log.DEBUG:
                logger.debug(msg)
            case Log.INFO:
                logger.info(msg)
            case Log.WARN:
                logger.warning(msg)
            case Log.ERROR:
                logger.error(msg)
            case Log.FATAL:
                logger.critical(msg)
            case other:
                logger.trace(f"log level was a weird value: {other}. msg: {msg}")

    @override
    def __hash__(self) -> int:
        return super().__hash__()


def main(args: list[str] | None = None):
    """
    Initializes the Node using `rcl`.
    """
    # let's also change some stuff about the logger
    adjust_logger()

    rclpy.init(args=args)

    log_node: LogNode = LogNode()
    rclpy.spin(log_node)

    log_node.destroy_node()
    rclpy.shutdown()


def adjust_logger():
    """
    Changes the Loguru format to be a bit easier to read.
    """
    # remove the existing subscriber
    logger.remove()

    # add new format
    fmt = "<light-black>{time:hh:mm:ss A}</light-black> <level>[{level}] {message}</level>"
    _ = logger.add(sys.stdout, format=fmt)

    # make colors closer to `tracing` :3
    _ = logger.level("DEBUG", color="<light-black>")
    _ = logger.level("INFO", color="<green>")
    _ = logger.level("WARNING", color="<yellow>")
    _ = logger.level("ERROR", color="<red>")
    _ = logger.level("CRITICAL", color="<RED><black>")


if __name__ == "__main__":
    main()
