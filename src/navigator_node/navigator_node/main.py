"""
stuff that the navigator node does:
- takes a coordinate and an indicator of which of the following we're doing: GPS, Aruco, or Object Detection
- turn lights RED (for Autonomous mode)
- no matter what, drive to a GPS coordinate (near a goal)
    - subscribe to GPS topic
    - receive messages repeatedly on current location
    - within class, save coordinate + time recv'd to use later
    - that includes distance calculation (how far are we from coordinate)
        - note: consider using WSG83 instead of flat coordinates for correct result
    - create a wheels publisher
    - based on distance, how far do we need to move, and in which direction?
        - PID controller (DO NOT IMPLEMENT MANUALLY @tyler)
    - send wheel speeds
- if Aruco, we need to subscribe to Aruco topic once coordinate is reached
    - this will return a pose of the marker
    - if marker found, we need to calculate distance to the marker
        - based on distance, we need to calculate how/where to move
    - if marker not found, we need to change strategy
- if Object Detection, we'll do something! but let's not care right now
- CALLBACK:
    - oh we received new gps info! now, let's move wheels, turn on lights, or exit
        - if we have reached coordinate and we are doing GPS, flash green lights, stop Rover
    - oh we received new aruco info! now, let's change strategy
- launches other nodes necessary for requested form of navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.publisher import Publisher

from typing import List
from dataclasses import dataclass
from enum import Enum

# TODO: move lights srv file to custom_interfaces
#
# this import will look like so: `from custom_interfaces import Lights`
from lights_node.srv import lights as LightsRequest
from custom_interfaces import GpsMessage, WheelsMessage, ArMessage
#
# FIXME: this is given by the `feat/sensors_node` branch, PR #36
from geographic_msgs import GeoPoint 


## how long we'll keep the data (DDS).
QUEUE_SIZE: int = 10

@dataclass(kw_only=True)
class NavigatorNode(Node):
    # NOTE: Why are these declared outside __init__?
    ## to make a publisher
    _wheels_publisher: Publisher
    ## to make our subscribers
    _gps_subscription: Subscription
    _ar_subscription: Subscription
    ## service client for lights node
    _client: Client

    def __init__(self):
        """
        Creates a new `NavigatorNode`.
        """
        super().__init__("Navigator Node")
        self.declare_parameter("navigator_parameters", None)

        #NOTE: this might not work
        param_value: NavigationParameters = self.get_parameter('navigator_parameters').value

        if param_value is None:
            self.get_logger().error("navigator_parameters is not set!")
            return
        try:
            self.nav_params = self.parse_navigation_parameters(param_value)
            self.get_logger().info(f"Loaded Navigation Parameters: {self.nav_params}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse navigator parameters: {e}")

        # create a service client for lights node
        self._client = self.create_client(
            srv_type = LightsRequest, 
            srv_name = "lights",
            callback_group = self._navigator_callback,
            qos_profile = QUEUE_SIZE,
        )

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # turn lights RED immediately upon startup
        lights_info = LightsRequest.Request()
        lights_info.red = 255
        lights_info.green = 0
        lights_info.blue = 0
        lights_info.flashing = False

        self.send_lights_request(lights_info)

        # create wheels publisher
        self._wheels_publisher = self.create_publisher(
            msg_type = WheelsMessage,
            topic = "/controls/wheels",
            qos_profile = QUEUE_SIZE,
        )

        # if in aruco mode, create a subscriber for aruco tracking
        if param_value.mode == NavigationMode.ARUCO:
            self._ar_subscription = self.create_subscription(
                msg_type = ArMessage, # TODO: change to wherever the ar message type is
                topic = "/aruco", #TODO: change to whatever the markers topic is
                callback=self._navigator_callback,
                qos_profile = QUEUE_SIZE,
            )

        # create a subscriber for GPS sensor
                # create the publisher
        self._gps_subscription = self.create_subscription(
            msg_type = GpsMessage,
            topic = "/sensors/gps",
            callback=self._navigator_callback,
            qos_profile = QUEUE_SIZE,
        )
    
    # Function to send lights request given a LightsRequest class instance
    def send_lights_request(self, lights_info: LightsRequest):
        """
        Send a request to the lights service to turn the lights red.
        """
        self.future = self._client.call_async(lights_info)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    # TODO: change msg type to whatever the gps message type is
    def _navigator_callback(self, _msg: GpsMessage | ArMessage):
        #TODO: implement this
        pass

class NavigationMode(Enum):
    GPS = 1,
    ARUCO = 2,
    OBJECT_DETECTION = 3,

@dataclass(kw_only=True)
class NavigationParameters():
    coord: GeoPoint
    """
    The coordinate that either is our goal (if `mode` is `GPS`), or is some
    point around an ArUco/object.
    """
    
    mode: NavigationMode
    """
    Indicates what the Navigator is doing. 
    
    For example, if we're given `NavigationMode::GPS`, we'll navigate to the given GPS
    coordinate and stop when we're there.
    """

def main(args: List[str] | None = None):
    """
    Initializes the Node using `rcl`.
    """
    rclpy.init(args=args)

    navigator_node: NavigatorNode = NavigatorNode()

    # spawn a task on the executor that continues running the Node until it's
    # destroyed.
    #
    # similar to the `tokio::spawn` syntax in Rust, but managed by `rcl` outside
    # the interpreter.
    #
    # slowdowns may occur, so consider benching anything questionable.
    rclpy.spin(navigator_node)

    # destroy the Node explicitly
    #
    # this is optional - otherwise, the garbage collector does it automatically
    # when it runs.
    navigator_node.destroy_node()
    rclpy.shutdown()


# runs the main function - Python doesn't do this automatically.
if __name__ == "__main__":
    main()

