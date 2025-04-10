"""
This module assists in estimating the location of goals as WSG84 coordinate
pairs.

Doing so lets us assign goals coordinates instead of trying to adjust using local
translations to the physical Rover.
"""

import math
import sys
from math import sqrt

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point, PoseStamped
from geopy.distance import distance
from loguru import logger as llogger


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
    _marker_position: Point = _pose.pose.position
    _marker_orientation = _pose.pose.orientation

    llogger.error("coordinate estimation is unimplemented!")
    sys.exit(1)


def get_distance_to_marker(marker: PoseStamped) -> float:
    """
    Given the pose information for an ArUco marker relative to the rover,
    calculate the distance to the marker
    """
    marker_position: Point = marker.pose.position
    distance_x: float = marker_position.x  # forward/backward distance
    distance_y: float = marker_position.y  # left/right distance

    # now we need to calculate the distance to the marker by taking the hypotenuse
    distance_to_marker: float = sqrt(distance_x**2 + distance_y**2)
    return distance_to_marker


def get_angle_to_target(
    dest_coord: GeoPoint,
    current_coord: GeoPointStamped,
    compass_z_degrees: float,
) -> float:
    """
    Calculate the angle from the robot to the destination.

    - `compass_z_degrees` must be normalized and within [0, 360] degrees.
    """
    unnormalized_angle = math.atan2(
        dest_coord.latitude - current_coord.position.latitude,
        dest_coord.longitude - current_coord.position.longitude,
    )
    # Normalize the angle to be between -pi and pi
    normalized_angle = (unnormalized_angle + math.pi) % (2 * math.pi) - math.pi

    error: float = normalized_angle - compass_z_degrees
    return error


def dist_m_between_coords(coord1: GeoPoint, coord2: GeoPoint) -> float:
    """
    Returns the distance between two coordinates, in meters.
    """
    dist_m = distance(
        [coord2.latitude, coord2.longitude],
        [coord1.latitude, coord1.longitude],
    ).meters

    # log and return
    llogger.trace(f"dist from coord 1 ({coord1}) and coord 2 ({coord2}) is: {dist_m}m")
    return dist_m
