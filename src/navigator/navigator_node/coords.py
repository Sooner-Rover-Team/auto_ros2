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


def calc_angle_to_target(
    dest_coord: GeoPoint,
    current_coord: GeoPointStamped,
    heading_degrees: float,
) -> float:
    """
    Calculate the angle from the robot to the destination.

    - `heading_degrees` must be normalized and within [0, 360] degrees.
    """
    llogger.trace(f"rover heading: {heading_degrees} deg")

    # Angle from the robot to the target (bearing)
    bearing_radians = math.atan2(
        dest_coord.longitude - current_coord.position.longitude,
        dest_coord.latitude - current_coord.position.latitude,
    )
    bearing_degrees = math.degrees(bearing_radians)

    # Calculate the angle needed to turn to the target
    error_degrees: float = bearing_degrees - heading_degrees

    # Normalize the error to be between -180 and 180
    normalized_error_degrees = ((error_degrees + 180) % 360) - 180

    return normalized_error_degrees


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


def generate_similar_coordinates(
    src: GeoPoint, radius: float, num_points: int
) -> list[GeoPoint]:
    """
    Given a coordinate, a radius in meters, and a number of points to return, this function
    takes the source coordinate and uses it to generate a list of new coordinates in
    a sequential radius around the starting coordinate.

    This functions as a simplistic "search" strategy for the rover, where we can then feed
    the generated coordinates as our desired path while we look for a tag or object.
    """
    new_points: list[GeoPoint] = []

    for i in range(num_points):
        angle = (360 / num_points) * i  # Evenly spaced angles around a circle

        # Calculates a new coordinate that is the given amount of meters away from source
        point: Point = distance(meters=radius).destination(
            Point(src.latitude, src.longitude), bearing=angle
        )

        # make that into a `GeoPoint` msg
        g: GeoPoint = GeoPoint()
        g.latitude = point.latitude
        g.longitude = point.longitude

        new_points.append(g)

    return new_points
