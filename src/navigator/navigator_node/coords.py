"""
This module assists in estimating the location of goals as WSG84 coordinate
pairs.

Doing so lets us assign goals coordinates instead of trying to adjust using local
translations to the physical Rover.
"""

import math
from math import sqrt

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point, Pose
from geopy.distance import distance
from geopy.point import Point as GeopyPoint
from loguru import logger as llogger


def coordinate_from_aruco_pose(
    current_location: GeoPoint, pose: Pose
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
    marker_position: Point = pose.position

    # `PoseStamped.position` from `aruco_node` is Rover-local translation in meters.
    #
    # We treat:
    # - `position.x` as North/South offset meters (+North),
    # - `position.y` as East/West offset meters (+East),
    #
    # then convert those meter offsets to geodesic `distance` + `bearing`.
    north_offset_m: float = marker_position.x
    east_offset_m: float = marker_position.y
    up_offset_m: float = marker_position.z

    # Convert Cartesian rover-local meters -> polar meters/degrees:
    #
    # - distance in meters from rover to marker
    # - bearing in degrees clockwise from North (what geopy expects)
    distance_to_marker_m: float = math.hypot(north_offset_m, east_offset_m)
    marker_bearing_deg: float = math.degrees(
        math.atan2(east_offset_m, north_offset_m)
    )

    # Convert current ROS `GeoPointStamped.position` -> geopy `Point` (lat, lon).
    current_geopy_point: GeopyPoint = GeopyPoint(
        current_location.latitude,
        current_location.longitude,
    )

    # Convert Rover coordinate + (distance, bearing) -> destination geodesic
    # point.
    estimated_geopy_point: GeopyPoint = distance(
        meters=distance_to_marker_m
    ).destination(current_geopy_point, bearing=marker_bearing_deg)

    # Convert geopy destination type -> ROS `GeoPoint`.
    estimated_marker_coord: GeoPoint = GeoPoint()
    estimated_marker_coord.latitude = estimated_geopy_point.latitude
    estimated_marker_coord.longitude = estimated_geopy_point.longitude

    # Preserve absolute altitude by adding local vertical offset in meters.
    estimated_marker_coord.altitude = current_location.altitude + up_offset_m

    return estimated_marker_coord


def get_distance_to_marker(marker: Pose) -> float:
    """
    Given the pose information for an ArUco marker relative to the rover,
    calculate the distance to the marker
    """
    marker_position: Point = marker.position
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
    llogger.trace(
        f"dist from coord 1 ({coord1}) and coord 2 ({coord2}) is: {dist_m}m"
    )
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
        point: GeopyPoint = distance(meters=radius).destination(
            GeopyPoint(src.latitude, src.longitude), bearing=angle
        )

        # make that into a `GeoPoint` msg
        g: GeoPoint = GeoPoint()
        g.latitude = point.latitude
        g.longitude = point.longitude

        new_points.append(g)

    return new_points
