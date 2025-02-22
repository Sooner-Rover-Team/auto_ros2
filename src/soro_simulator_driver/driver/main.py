from dataclasses import dataclass

import rclpy
from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Vector3
from loguru import logger as llogger

# FIXME: this (the "python" namespace) is horrifying but im trying
#        it's caused by Webots lacking a `pyproject.toml` for their python
#        bindings :p
from python.controller.robot import GPS as Gps
from python.controller.robot import LED as Led
from python.controller.robot import (
    Accelerometer,
    Camera,
    Compass,
    Gyro,
    Motor,
    Robot,
)
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu, NavSatFix

from build.custom_interfaces.rosidl_generator_py.custom_interfaces.msg._gps_message import (
    GpsMessage,
)
from custom_interfaces.msg._imu_message import ImuMessage
from custom_interfaces.msg._wheels_message import WheelsMessage
from custom_interfaces.srv._lights import Lights
from custom_interfaces.srv._lights import Lights_Request as LightsRequest
from custom_interfaces.srv._lights import Lights_Request as LightsResponse
from log_node.main import QOS_PROFILE
from src.soro_simulator_driver.driver.wheel_speeds import translate_u8


@dataclass(kw_only=True)
class SoroDriver(Robot):
    __node: Node
    __robot: Robot

    __led: Led
    """
    The LED that indicates the Rover's current mode.

    Required because of URC rules.
    """
    __gyro: Gyro
    """Sensor that indicates rotation."""
    __accel: Accelerometer
    """Sensor that indicates acceleration."""
    __compass: Compass
    """Sensor that shows what direction the Rover is facing."""
    __depth_camera: Camera
    """FIXME: replace with something else"""
    __mono_camera: Camera
    """The primary camera used on the Rover."""
    __gps: Gps
    """A sensor that indicates our position in the world."""

    # motors... there are quite a few
    __left_front_wheel_motor: Motor
    __left_middle_wheel_motor: Motor
    __left_back_wheel_motor: Motor
    __right_front_wheel_motor: Motor
    __right_middle_wheel_motor: Motor
    __right_back_wheel_motor: Motor

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
        super(SoroDriver, self).__init__()

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_front_wheel_motor = self.__robot.getMotor(
            "left_front_wheel_motor"
        )
        self.__left_middle_wheel_motor = self.__robot.getMotor(
            "left_middle_wheel_motor"
        )
        self.__left_back_wheel_motor = self.__robot.getMotor(
            "left_back_wheel_motor"
        )
        self.__right_front_wheel_motor = self.__robot.getMotor(
            "right_front_wheel_motor"
        )
        self.__right_middle_wheel_motor = self.__robot.getMotor(
            "right_middle_wheel_motor"
        )
        self.__right_back_wheel_motor = self.__robot.getMotor(
            "right_back_wheel_motor"
        )

        rclpy.init(args=None)
        self.__node = rclpy.create_node("my_robot_driver")

        # make publishers for each (fake) sensor.
        #
        # in a way, this is kinda just another `sensors_node`.
        #
        # gps
        self.__gps_publisher = self.__node.create_publisher(
            GpsMessage, "/sensors/gps", QOS_PROFILE
        )
        self.__sim_gps_subscriber = self.__node.create_subscription(
            NavSatFix, "/sim/gps", self.sim_gps_callback, QOS_PROFILE
        )
        # imu
        self.__imu_publisher = self.__node.create_publisher(
            ImuMessage, "/sensors/imu", QOS_PROFILE
        )
        self.__sim_imu_subscriber = self.__node.create_subscription(
            Imu, "/sim/imu", self.sim_imu_callback, QOS_PROFILE
        )

        # depth camera
        # self.__node.create_publisher(Image, "/sensors/depth_image", QOS_PROFILE)
        # mono camera
        # self.__node.create_publisher(Image, "/sensors/mono_image", QOS_PROFILE)

        # react to controls
        self.__wheels_subscriber = self.__node.create_subscription(
            WheelsMessage, "/control/wheels", self.wheels_callback, QOS_PROFILE
        )
        self.__lights_service = self.__node.create_service(
            Lights, "/control/lights", self.lights_callback
        )

    pass

    def lights_callback(
        self, req: LightsRequest, resp: LightsResponse
    ) -> LightsResponse:
        llogger.debug(f"recv'd lights request! see: {req}")

        # change the color of the lights
        lights_value: int = 0x000000
        lights_value += req.red << 16
        lights_value += req.green << 8
        lights_value += req.blue
        self.__led.set(lights_value)

        return resp  # FIXME: add fields

    def wheels_callback(self, msg: WheelsMessage):
        llogger.debug(f"recv'd wheel speeds! see: {msg}")

        BASE_SPEED: float = 1.0
        left_wheel_speed: float = BASE_SPEED * translate_u8(msg.left_wheels)
        right_wheel_speed: float = BASE_SPEED * translate_u8(msg.right_wheels)

        # set left wheel speeds
        self.__left_front_wheel_motor.setVelocity(left_wheel_speed)
        self.__left_middle_wheel_motor.setVelocity(left_wheel_speed)
        self.__left_back_wheel_motor.setVelocity(left_wheel_speed)

        # set right wheel speeds
        self.__right_front_wheel_motor.setVelocity(right_wheel_speed)
        self.__right_middle_wheel_motor.setVelocity(right_wheel_speed)
        self.__right_back_wheel_motor.setVelocity(right_wheel_speed)
        pass

    def sim_gps_callback(self, msg: NavSatFix):
        """immediately publishes to the `/sensors/gps` topic after translating"""
        translated: GeoPoint = GeoPoint()
        translated.latitude = msg.latitude
        translated.longitude = msg.longitude
        translated.altitude = msg.altitude
        translated_stamped: GeoPointStamped = GeoPointStamped()
        translated_stamped.position = translated

        llogger.debug(f"publishing gps info: {translated}")
        self.__gps_publisher.publish(translated_stamped)

    def sim_imu_callback(self, msg: Imu):
        """
        translates and immediately publishes to `/sensors/imu`.
        """
        translated: ImuMessage = ImuMessage()

        # accel
        translated.accel = msg.linear_acceleration

        # compass
        translated.compass = Vector3()
        translated.compass.x = msg.orientation.x
        translated.compass.y = msg.orientation.y
        translated.compass.z = msg.orientation.z

        # gyro
        translated.gyro = msg.angular_velocity

        # publish the translated message
        llogger.debug(f"publishing imu info: {translated}")
        self.__imu_publisher.publish(translated)


def main():
    llogger.info("starting simulator driver...")

    # FIXME: i think this might need to be called by the `webots` cli, but
    #        maybe ask tyler..?
    # driver: SoroDriver = SoroDriver()


if __name__ == "__main__":
    main()
