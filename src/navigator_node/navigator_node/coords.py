"""
This module assists in estimating the location of goals as WSG84 coordinate
pairs.

Doing so lets us assign goals coordinates instead of trying to adjust using local
translations to the physical Rover.
"""

import math
import sys

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point, PoseStamped
from loguru import logger as llogger
from custom_interfaces.msg import ImuMessage
from tf_transformations import euler_from_quaternion


# Calculate the error
def get_angle_to_dest(
    current_coord: GeoPointStamped,
    dest_coord: GeoPoint,
    current_imu: ImuMessage,
) -> float:
    """Calculate the angle from the robot to the destination."""
    # Convert latitude and longitude to radians
    curr_lat, curr_long, dest_lat, dest_long = map(
        math.radians,
        [
            current_coord.position.latitude,
            current_coord.position.longitude,
            dest_coord.latitude,
            dest_coord.longitude,
        ],
    )

    long_diff = dest_long - curr_long

    # From dest or from curr?
    #y = math.sin(long_diff) * math.cos(curr_lat)
    y = math.sin(long_diff) * math.cos(dest_lat)
    x = math.cos(curr_lat) * math.sin(dest_lat) - math.sin(curr_lat) * math.cos(
        dest_lat
    ) * math.cos(long_diff)

    target_bearing = math.atan2(y, x)  # in radians
    target_bearing = math.degrees(target_bearing) % 360  # convert to degrees

    # Get the rover's current angle
    # NOTE: This assumes that the compass is level at all times
    bearing_deg = math.degrees(math.atan2(current_imu.compass.y, current_imu.compass.x))
    rover_bearing = (bearing_deg + 360) % 360

    llogger.debug(f"Rover Bearing: {rover_bearing}")

    # Find the difference between bearing to destination and current orientation
    #error = (rover_bearing + target_bearing) % 360  # Wrap between -180 and 180
    error = (target_bearing - rover_bearing) % 360
    if error > 180:
        error -= 360
    # do we need to add 360 somewhere?
    return error


# Calculate the error
def get_angle_to_dest_original(
    current_coord: GeoPointStamped,
    dest_coord: GeoPoint,
    current_imu: ImuMessage,
) -> float:
    """Calculate the angle from the robot to the destination."""
    # Convert latitude and longitude to radians
    curr_lat, curr_long, dest_lat, dest_long = map(
        math.radians,
        [
            current_coord.position.latitude,
            current_coord.position.longitude,
            dest_coord.latitude,
            dest_coord.longitude,
        ],
    )

    long_diff = dest_long - curr_long

    # SHOULD Y AND X BE SWITCHED?
    y = math.sin(long_diff) * math.cos(dest_lat)
    x = math.cos(curr_lat) * math.sin(dest_lat) - math.sin(curr_lat) * math.cos(
        curr_lat
    ) * math.cos(long_diff)

    target_bearing = math.atan2(y, x)  # in radians
    target_bearing = math.degrees(target_bearing) % 360  # convert to degrees

    # Get the rover's current angle
    # TODO: Change this to use compass information instead of IMU
    _, _, yaw = euler_from_quaternion(
        [
            current_imu.orientation.x,
            current_imu.orientation.y,
            current_imu.orientation.z,
            current_imu.orientation.w,
        ]
    )

    rover_bearing = math.degrees(yaw)
    llogger.debug(f"Rover Bearing: {rover_bearing}")

    # Find the difference between bearing to destination and current orientation
    error = (
        target_bearing - rover_bearing + 180
    ) % 360 - 180  # Wrap between -180 and 180
    return error


def coordinate_from_aruco_pose(
    _current_location: GeoPointStamped, _pose: PoseStamped
) -> GeoPoint:
    """
    Given the Rover's current location and the ArUco marker's current pose,
    this function calculates an approximate coordinate for the marker.

    These coordinates allow the Navigator to start moving toward an ArUco
    marker.

    If we can get the current heading/bearing of the rover from compass info,
    we can use that and the distance to the marker to calculate the estimated
    coordinate.
    """
    marker_position: Point = _pose.pose.position
    marker_orientation = _pose.pose.orientation

    llogger.error("coordinate estimation is unimplemented!")
    sys.exit(1)


def get_distance_to_marker(marker: PoseStamped) -> float:
    """
    Given the pose information for an ArUco marker relative to the rover,
    calculate the distance to the marker
    """
    marker_position: Point = marker.pose.point
    distance_x: float = marker_position.x  # forward/backward distance
    distance_y: float = marker_position.y  # left/right distance

    # now we need to calculate the distance to the marker by taking the hypotenuse
    distance_to_marker: float = math.sqrt(distance_x**2 + distance_y**2)
    return distance_to_marker
