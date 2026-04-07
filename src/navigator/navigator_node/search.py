"""
Implementation of the target search algorithm.
"""

from geographic_msgs.msg import GeoPoint
from geopy.distance import distance
from geopy.point import Point
from loguru import logger as llogger
from rclpy.publisher import Publisher


# FIXME: this needs to be a class method. we can't just navigate without `self` :(
async def search_for_target(
    lead_coordinates: list[GeoPoint], _wheel_publisher: Publisher
):
    """
    Moves the Rover to each potential target location we've collected. The info
    obtained from performing this search isn't managed by this function.

    Instead, other ROS 2 nodes will tell the Navigator that it has found
    something, in which case it can cancel this async function.

    IMPORTANT: Ensure that you don't re-use this list (or that it has been
    deep-copied)! We might modify it.
    """
    llogger.debug("starting search for a list of leads...")

    # iterate over each "potential location" (where a target would be located)
    for potential_location in lead_coordinates:
        llogger.debug(
            f"checking location at ({potential_location.latitude} lat, {potential_location.longitude} lon)..."
        )
        pass
    pass


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
