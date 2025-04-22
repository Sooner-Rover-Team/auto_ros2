import math
import signal
import sys
from dataclasses import dataclass
from time import sleep

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import Vector3
from gps_msgs.msg import GPSFix as GpsFix
from loguru import logger as llogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import (
    QoSPresetProfiles,
    QoSProfile,
)
from rclpy.service import Service
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from typing_extensions import override

from custom_interfaces.msg._imu_message import ImuMessage
from custom_interfaces.msg._wheels_message import WheelsMessage
from custom_interfaces.srv._lights import Lights
from custom_interfaces.srv._lights import Lights_Request as LightsRequest
from custom_interfaces.srv._lights import Lights_Response as LightsResponse

from .wheel_speeds import translate_u8

QOS_PROFILE: QoSProfile = QoSPresetProfiles.SYSTEM_DEFAULT.value


@dataclass(kw_only=True)
class SoroBridge(Node):
    """Translates stuff from the simulator into the messages we use."""

    # motors... there are quite a few.
    #
    # these are directly from gazebo
    __left_front_wheel_motor: Publisher
    __left_middle_wheel_motor: Publisher
    __left_back_wheel_motor: Publisher
    __right_front_wheel_motor: Publisher
    __right_middle_wheel_motor: Publisher
    __right_back_wheel_motor: Publisher

    # interface speakers/listeners
    __lights_service: Service
    __wheels_subscriber: Subscription
    __gps_publisher: Publisher
    __imu_publisher: Publisher
    __depth_camera_publisher: Publisher
    __mono_camera_publisher: Publisher

    # fake sim interface speakers/listeners...
    #
    # these are used to publish messages like the `sensors_node` :D
    __sim_imu_subscriber: Subscription
    # __sim_gyro_subscriber: Subscription
    # __sim_accel_subscriber: Subscription
    # __sim_compass_subscriber: Subscription
    __sim_gps_subscriber: Subscription

    def __init__(self):
        # initialize the `Robot` superclass
        super().__init__("soro_bridge")

        self.__gps_publisher = self.create_publisher(
            GeoPointStamped, "/sensors/gps", QOS_PROFILE
        )
        self.__sim_gps_subscriber = self.create_subscription(
            GpsFix, "/sim/gps", self.sim_gps_callback, QOS_PROFILE
        )
        self.__imu_publisher = self.create_publisher(
            ImuMessage, "/sensors/imu", QOS_PROFILE
        )
        self.__sim_imu_subscriber = self.create_subscription(
            Imu, "/sim/imu", self.sim_imu_callback, QOS_PROFILE
        )

        # react to controls
        self.__wheels_subscriber = self.create_subscription(
            WheelsMessage, "/control/wheels", self.wheels_callback, QOS_PROFILE
        )
        self.__lights_service = self.create_service(
            Lights, "/control/lights", self.lights_callback
        )

        # make wheel publishers
        self.__left_front_wheel_motor = self.create_publisher(
            Float64, "/sim/left_front_wheel/vel", QOS_PROFILE
        )
        self.__left_middle_wheel_motor = self.create_publisher(
            Float64, "/sim/left_middle_wheel/vel", QOS_PROFILE
        )
        self.__left_back_wheel_motor = self.create_publisher(
            Float64, "/sim/left_back_wheel/vel", QOS_PROFILE
        )
        self.__right_front_wheel_motor = self.create_publisher(
            Float64, "/sim/right_front_wheel/vel", QOS_PROFILE
        )
        self.__right_middle_wheel_motor = self.create_publisher(
            Float64, "/sim/right_middle_wheel/vel", QOS_PROFILE
        )
        self.__right_back_wheel_motor = self.create_publisher(
            Float64, "/sim/right_back_wheel/vel", QOS_PROFILE
        )

    @override
    def __hash__(self) -> int:
        return super().__hash__()

    def lights_callback(
        self, req: LightsRequest, resp: LightsResponse
    ) -> LightsResponse:
        llogger.debug(f"recv'd lights request! see: {req}")

        # change the color of the lights
        lights_value: int = 0x000000
        lights_value += req.red << 16
        lights_value += req.green << 8
        lights_value += req.blue
        # self.__led.set(lights_value) # FIXME

        return resp  # FIXME: add fields

    def wheels_callback(self, msg: WheelsMessage):
        llogger.debug(f"recv'd wheel speeds! see: {msg}")

        BASE_SPEED: float = 0.2
        left_wheel_speed: float = BASE_SPEED * translate_u8(msg.left_wheels)
        right_wheel_speed: float = BASE_SPEED * translate_u8(msg.right_wheels)

        # make float64 msgs
        left_msg: Float64 = Float64()
        left_msg.data = left_wheel_speed
        right_msg: Float64 = Float64()
        right_msg.data = right_wheel_speed

        # don't send wheel speeds before we have subscribers
        while (
            self.__left_back_wheel_motor.get_subscription_count() == 0
            or self.__left_middle_wheel_motor.get_subscription_count() == 0
            or self.__left_front_wheel_motor.get_subscription_count() == 0
            or self.__right_back_wheel_motor.get_subscription_count() == 0
            or self.__right_middle_wheel_motor.get_subscription_count() == 0
            or self.__right_front_wheel_motor.get_subscription_count() == 0
        ):
            llogger.debug("won't send speeds to sim until motor bridge is up")
            sleep(0.25)

        # set left wheel speeds
        _ = self.get_logger().debug(f"left wheel speed: {left_wheel_speed}")
        self.__left_front_wheel_motor.publish(left_msg)
        self.__left_middle_wheel_motor.publish(left_msg)
        self.__left_back_wheel_motor.publish(left_msg)

        # set right wheel speeds
        _ = self.get_logger().debug(f"right wheel speed: {right_wheel_speed}")
        self.__right_front_wheel_motor.publish(right_msg)
        self.__right_middle_wheel_motor.publish(right_msg)
        self.__right_back_wheel_motor.publish(right_msg)

    def sim_gps_callback(self, msg: NavSatFix):
        """immediately publishes to the `/sensors/gps` topic after translating"""
        translated: GeoPointStamped = GeoPointStamped()

        translated.position.latitude = msg.latitude
        translated.position.longitude = msg.longitude
        translated.position.altitude = msg.altitude

        llogger.debug(f"publishing gps info: {translated}")
        self.__gps_publisher.publish(translated)

    def sim_imu_callback(self, msg: Imu):
        """
        publishes custom imu message from the ros 2 one (sensor_msgs/msg/Imu)!
        """
        translated: ImuMessage = ImuMessage()

        translated.accel = msg.linear_acceleration
        translated.gyro = msg.angular_velocity
        translated.temp_c = 0.0

        # get compass in deg from quat.
        #
        # we're gonna assume +z is forward, so it's just typical 3d space
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # this is the rotation on the z-axis
        #
        # from wikipedia:
        #
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - (2.0 * (qy * qy + qz * qz)))

        # convert it into degrees
        compass_degrees = (math.degrees(yaw) + 360) % 360

        # FIXME(bray): make this just one value
        #
        # ...to save my poor innocent heart from more nonsense
        translated.compass = Vector3()
        translated.compass.x = 0.0
        translated.compass.y = 0.0
        translated.compass.z = compass_degrees

        # TODO: remove
        llogger.warning(f"compass degrees: {compass_degrees}")

        self.__imu_publisher.publish(translated)


def main(args: list[str] | None = None):
    # exit gracefully when Ctrl^C'd
    def exit_handler(_handler: int, _):
        llogger.info("Asked to shutdown! Doing so gracefully...")
        rclpy.try_shutdown()
        sys.exit(0)

    _ = signal.signal(signal.SIGINT, exit_handler)
    _ = signal.signal(signal.SIGTERM, exit_handler)

    llogger.info("Starting simulator driver...")
    rclpy.init(args=args)

    bridge_node: SoroBridge = SoroBridge()
    llogger.info("Simulator has been initialized.")

    # spawn a task on the executor that continues running the Node until it's
    # destroyed.
    #
    # similar to the `tokio::spawn` syntax in Rust, but managed by `rcl` outside
    # the interpreter.
    #
    # slowdowns may occur, so consider benching anything questionable.
    llogger.info("Simulator is spinning...")
    rclpy.spin(bridge_node)

    # destroy the Node explicitly
    #
    # this is optional - otherwise, the garbage collector does it automatically
    # when it runs.
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.try_shutdown()


# runs the main function - Python doesn't do this automatically.
if __name__ == "__main__":
    main()
