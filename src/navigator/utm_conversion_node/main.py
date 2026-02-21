import sys
from dataclasses import dataclass

import rclpy
from custom_interfaces.srv import GnssToMap
from custom_interfaces.srv._gnss_to_map import (
    GnssToMap_Request,
    GnssToMap_Response,
)
from geodesy import utm
from geodesy.utm import UTMPoint as UtmPoint
from geometry_msgs.msg import Point
from loguru import logger as llogger
from nav_msgs.msg import Odometry
from rclpy.node import Node, Service, Subscription
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSPresetProfiles,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix
from typing_extensions import override

GPS_QOS_PROFILE: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)

ODOM_QOS_PROFILE: QoSProfile = QoSPresetProfiles.SENSOR_DATA.value
ODOM_QOS_PROFILE.reliability = QoSReliabilityPolicy.RELIABLE
ODOM_QOS_PROFILE.depth = 10


@dataclass(kw_only=True)
class UtmConversionNode(Node):
    _gps_subscriber: Subscription
    _odom_subscriber: Subscription
    _last_known_odom: Odometry | None = None
    _last_known_utm: UtmPoint | None = None
    _gnss_coord_to_map_coord_service: Service

    def __init__(self):
        super().__init__("utm_conversion_node")

        # grab our gps conn
        self._gps_subscriber = self.create_subscription(
            NavSatFix,
            "/sensors/gps",
            self._on_filtered_gps_message,
            GPS_QOS_PROFILE,
        )

        # also, grab odom stream
        self._odom_subscriber = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self._on_odom_message,
            ODOM_QOS_PROFILE,
        )

        # finally, make a service to convert gnss coords -> map coords
        self._gnss_coord_to_map_coord_service = self.create_service(
            GnssToMap, "convert_gps_to_map", self._handle_gnss_conversion
        )

    @override
    def __hash__(self) -> int:
        return super().__hash__()

    def _on_filtered_gps_message(self, msg: NavSatFix):
        self._last_known_utm = utm.fromLatLong(msg.latitude, msg.longitude)

    def _on_odom_message(self, msg: Odometry):
        self._last_known_odom = msg

    def _handle_gnss_conversion(
        self, req: GnssToMap_Request, resp: GnssToMap_Response
    ) -> GnssToMap_Response:
        # if we don't have the offset, early return...
        if self._last_known_odom is None:
            llogger.warning(
                "srv called, but `odom` (`/odometry/filtered`) isn't available yet. \
                try again in a moment..."
            )
            resp.success = False
            return resp
        if self._last_known_utm is None:
            llogger.warning(
                "srv called, and `odom` is available, but `utm` (`/sensors/gps`) isn't available yet. \
                try again in a moment..."
            )
            resp.success = False
            return resp

        # convert the given geopoint into a utm
        target_utm: UtmPoint = utm.fromLatLong(
            req.gnss_coord_to_convert.latitude,
            req.gnss_coord_to_convert.longitude,
        )

        # find the calculate our n/e dy + dx
        east_diff: float = target_utm.easting - self._last_known_utm.easting
        north_diff: float = target_utm.northing - self._last_known_utm.northing

        # grab our position offset from the odometry
        odom_pos_offset: Point = self._last_known_odom.pose.pose.position

        # create a point in the path response
        point: Point = Point()
        point.x = odom_pos_offset.x + east_diff
        point.y = odom_pos_offset.y + north_diff
        point.z = 0.0

        # respond with the point
        resp.point = point
        resp.success = True
        return resp


def main(args: list[str] | None = None):
    llogger.info("Starting UTM conversion Node...")
    rclpy.init(args=args)
    node: UtmConversionNode = UtmConversionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        llogger.info("Ctrl^C detected. Shutting down gracefully...")
    finally:
        stop(node)


def stop(node: UtmConversionNode):
    llogger.info("Destroying node. Goodbye!")
    node.destroy_node()

    rclpy.shutdown()
    llogger.info("Exiting...")
    sys.exit(0)


if __name__ == "__main__":
    main()
