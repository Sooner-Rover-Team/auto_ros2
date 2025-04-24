from dataclasses import dataclass
from enum import Enum

from geographic_msgs.msg import GeoPoint


class NavigationMode(Enum):
    GPS = 0
    ARUCO = 1
    OBJECT_DETECTION = 2


class GoToCoordinateReason(Enum):
    """
    The 'reason' we're going to a coordinate.

    As an argument to `go_to_coordinate`, this type allows it to use the correct
    stopping distance.
    """

    GPS = 0
    ARUCO = 1


# these are used by the PID controller... if not given through the ROS 2
# parameters!
DEFAULT_PID_PROPORTIONAL_GAIN: float = 0.08
DEFAULT_PID_INTEGRAL_GAIN: float = 0.04
DEFAULT_PID_DERIVATIVE_GAIN: float = 0.08


@dataclass(kw_only=True)
class NavigationParameters:
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
    pk: float = DEFAULT_PID_PROPORTIONAL_GAIN
    """
    Proportional gain for the PID controller.
    Determines the speed at which the rover will correct itself.
    Default: 0.0
    """
    pi: float = DEFAULT_PID_INTEGRAL_GAIN
    """
    Integral gain for the PID controller.
    Determines how much the rover will correct itself.
    Default: 0.0
    """
    pd: float = DEFAULT_PID_DERIVATIVE_GAIN
    """
    Derivative gain for the PID controller.
    Determines how quickly the rover will stop correcting itself.
    Default: 0.0
    """
