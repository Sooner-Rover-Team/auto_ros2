from geographic_msgs.msg import GeoPoint

from navigator_node.coords import calc_angle_to_target


def make_geopoint(lat: float, lon: float, alt: float) -> GeoPoint:
    """constructs a new geopoint. the args are in meters"""
    pt: GeoPoint = GeoPoint()
    pt.latitude = lat
    pt.longitude = lon
    pt.altitude = alt

    return pt

# takes long, lat (x,y)
def calc_angle(
    target_longitude: float,
    target_latitude: float,
    robot_heading: float,
) -> float:
    """Calculate the angle from the robot to the destination."""
    # coordinate of the robot
    robot_coord: GeoPoint = make_geopoint(0.0, 0.0, 0.0)

    # coordinate of some {location, tag, object}
    target_coord: GeoPoint = make_geopoint(
        target_latitude, target_longitude, 0.0
    )

    return calc_angle_to_target(target_coord, robot_coord, robot_heading)


def test_calc_target_north():
    """
    When the robot is at origin and the target is North.
    """

    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [0, -45, -90, -135, -180, 135, 90, 45, 0] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
       assert calc_angle(0.0, 1.0, robot_heading) == expected_error # far away target
       assert calc_angle(0.0, 0.0001, robot_heading) == expected_error # close target

def test_calc_target_northeast():
    # Simulate the target being Northeast and the robot being at different directions to the target
    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [45, 0, -45, -90, -135, -180, 135, 90, 45] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
       assert calc_angle(1.0, 1.0, robot_heading) == expected_error # far away target
       assert calc_angle(0.0001, 0.0001, robot_heading) == expected_error # close target

def test_calc_target_east():
    # Simulate the target being East and the robot being at different directions to the target
    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [90, 45, 0, -45, -90, -135, -180, 135, 90] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
        assert calc_angle(1.0, 0.0, robot_heading) == expected_error # far away target
        assert calc_angle(0.0001, 0.0, robot_heading) == expected_error # close target

def test_calc_target_southeast():
    # Simulate the target being Southeast and the robot being at different directions to the target
    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [135, 90, 45, 0, -45, -90, -135, -180, 135] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
        assert calc_angle(1.0, -1.0, robot_heading) == expected_error # far away target
        assert calc_angle(0.0001, -0.0001, robot_heading) == expected_error # close target

def test_calc_target_south():
    # Simulate the target being South and the robot being at different directions to the target
    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [-180, 135, 90, 45, 0, -45, -90, -135, -180] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
        assert calc_angle(0.0, -1.0, robot_heading) == expected_error # far away target
        assert calc_angle(0.0, -0.0001, robot_heading) == expected_error # close target

def test_calc_target_southwest():
    # Simulate the target being South and the robot being at different directions to the target
    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [-135, -180, 135, 90, 45, 0, -45, -90, -135] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
        assert calc_angle(-1.0, -1.0, robot_heading) == expected_error # far away target
        assert calc_angle(-0.0001, -0.0001, robot_heading) == expected_error # close target

def test_calc_target_west():
    # Simulate the target being South and the robot being at different directions to the target
    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [-90, -135, -180, 135, 90, 45, 0, -45, -90] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
        assert calc_angle(-1.0, 0.0, robot_heading) == expected_error # far away target
        assert calc_angle(-0.0001, 0.0, robot_heading) == expected_error # close target

def test_calc_target_northwest():
    # Simulate the target being South and the robot being at different directions to the target
    robot_headings = [degree for degree in range(0, 361, 45)] # degrees from 0 to 360
    expected_errors = [-45, -90, -135, -180, 135, 90, 45, 0, -45] # in degrees
    for robot_heading, expected_error in zip(robot_headings, expected_errors):
        assert calc_angle(-1.0, 1.0, robot_heading) == expected_error # far away target
        assert calc_angle(-0.0001, 0.0001, robot_heading) == expected_error # close target
