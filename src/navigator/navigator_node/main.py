import asyncio
import sys
from dataclasses import dataclass
from queue import PriorityQueue

import rclpy
from rclpy import action
from custom_interfaces.srv import GnssToMap, Lights
from custom_interfaces.srv._gnss_to_map import GnssToMap_Response
from custom_interfaces.srv._lights import (
    Lights_Request as LightsRequest,
)
from custom_interfaces.srv._lights import (
    Lights_Response as LightsResponse,
)
from builtin_interfaces.msg import Time
from custom_interfaces.action import FindArucoWithPose

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point, PoseStamped, Twist
from loguru import logger as llogger
from nav2_simple_commander.robot_navigator import BasicNavigator
from rcl_interfaces.msg import ParameterType
from rclpy.client import Client
from rclpy.node import (
    Node,
    ParameterDescriptor,
)
from rclpy.action.client import ActionClient
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rclpy.subscription import Subscription
from sensor_msgs.msg import NavSatFix
from typing_extensions import override

from .coords import dist_m_between_coords, generate_similar_coordinates, coordinate_from_aruco_pose, get_distance_to_marker
from .pose import geopoint_to_pose
from .types import (
    NavigationMode,
    NavigationParameters,
)

SENSORS_QOS_PROFILE: QoSProfile = QoSPresetProfiles.SENSOR_DATA.value
"""
A quality of service (QoS) profile optimized for sensors.

Used across Autonomous systems.
"""

RELIABLE_QOS_PROFILE: QoSProfile = QoSPresetProfiles.SERVICES_DEFAULT.value
"""
A "reliable" QoS service for things like sending messages to the wheels.
"""

## minimum distance we need to be within from the coordinate
MIN_GPS_DISTANCE: float = 3.0  # meters
MIN_ARUCO_DISTANCE: float = 2.0  # meters


@dataclass(kw_only=True)
class NavigatorNode(Node):
    _cmd_vel_publisher: Publisher
    """
    Publish pose transformations to move the Rover.

    Since Nav2 handles this automatically, we only use it to stop the wheels
    manually (more as a safety thing - a sanity check).
    """
    _gps_subscription: Subscription
    """A subscriber to the GPS (NavSat) to know where the Rover is."""
    _aruco_subscription: Subscription # TODO: remove this
    """Subscription to the `aruco_node` to understand what it's found so far."""

    _aruco_client: ActionClient
    """An action client for the `aruco_node` to understand what it's found so far and receive feedback."""

    _lights_client: Client
    """
    Service client for the Lights node. We use it to tell the `lights_node` to
    change the lights.
    """

    nav_parameters: NavigationParameters
    """
    A type containing all the parameters we got when the node was launched.

    Useful for storing stuff like the marker we're navigating to.
    """
    _given_aruco_marker_id: int | None = None
    """
    The ID of the ArUco marker we're looking for.

    We don't use this directly - it's only to make error messages better.
    """

    _last_known_rover_coord: GeoPointStamped | None = None
    """The coordinate last recv'd from the GPS."""
    _last_known_marker_coord: GeoPointStamped | None = None
    """Coordinate pair where the target was last known to be located."""
    _gps_to_map_client: Client
    """A client to speak with the `utm_conversion_node`."""

    _aruco_action_feedback: FindArucoWithPose.Feedback | None = None
    """A variable to store the latest feedback from the ArUco action server."""

    _curr_marker_transform: PoseStamped | None = None
    """
    The location of the ArUco marker relative to the Rover.

    TODO: make this use a UTM coordinate or something; we can't be local to the
    Rover when it's moving lol
    """

    def __init__(self):
        """
        Creates a new `NavigatorNode`.
        """
        super().__init__("navigator_node")

        _ = self.get_logger().info("Starting Navigator...")

        # declare each parameter as required by ROS 2...
        #
        # coord
        latitude_desc: ParameterDescriptor = ParameterDescriptor()
        latitude_desc.description = "latitude for coordinate we'll nav to"
        latitude_desc.type = ParameterType.PARAMETER_DOUBLE
        _ = self.declare_parameter(name="latitude", descriptor=latitude_desc)
        longitude_desc: ParameterDescriptor = ParameterDescriptor()
        longitude_desc.description = "longitude for coordinate we'll nav to"
        longitude_desc.type = ParameterType.PARAMETER_DOUBLE
        _ = self.declare_parameter(name="longitude", descriptor=longitude_desc)

        # mode
        mode_desc: ParameterDescriptor = ParameterDescriptor()
        mode_desc.description = "the thing we're doing right now. i.e. to do \
            aruco tracking, use NavigationMode.ARUCO"
        mode_desc.type = ParameterType.PARAMETER_INTEGER
        _ = self.declare_parameter(name="mode", descriptor=mode_desc)

        # try to grab the instructions we're given over parameters.
        #
        # if none is given, this is `None`, and will cause the program to go
        # down
        latitude: float | None = self.get_parameter("latitude").value
        longitude: float | None = self.get_parameter("longitude").value
        if latitude is None:
            _ = self.get_logger().error(
                "The `latitude` parameter is not set! The navigator will now exit."
            )
            sys.exit(1)
        if longitude is None:
            _ = self.get_logger().error(
                "The `longitude` parameter is not set! The navigator will now exit."
            )
            sys.exit(1)

        # construct coordinate from lat + long
        coord: GeoPoint = GeoPoint()
        coord.latitude = latitude
        coord.longitude = longitude
        coord.altitude = 0.0

        mode_int: int | None = self.get_parameter("mode").value
        if mode_int is None:
            _ = self.get_logger().error(
                "The `coord` parameter is not set! The navigator will now exit."
            )
            sys.exit(1)

        # construct the parameters
        self.nav_parameters = NavigationParameters(
            coord=coord,
            mode=NavigationMode(mode_int),
        )
        _ = self.get_logger().debug("constructed all parameters.")

        # create a service client for lights node
        self._lights_client = self.create_client(
            srv_type=Lights,
            srv_name="/control/lights",
        )

        # wait for the lights service to come up
        llogger.debug("Attempting to connect to lights service...")
        _ = self._lights_client.wait_for_service()
        llogger.debug("Lights service is up!")

        # turn lights RED immediately upon startup.
        #
        # it's required for competition :)
        lights_info: LightsRequest = LightsRequest()
        lights_info.red = 255
        lights_info.green = 0
        lights_info.blue = 0
        lights_info.flashing = False

        _ = self.send_lights_request(lights_info)

        # create wheels publisher
        self._cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic="/cmd_vel",
            qos_profile=RELIABLE_QOS_PROFILE,
        )

        # if in aruco mode, create a subscriber for aruco tracking
        if self.nav_parameters.mode == NavigationMode.ARUCO:
            llogger.info("We're in ArUco mode. Creating ArUco client...")
            self._aruco_client = ActionClient(
                node=self,
                action_type=FindArucoWithPose,
                action_name="find_aruco_with_pose"
            )

        # connect to our sensors using subscriptions
        self._gps_subscription = self.create_subscription(
            msg_type=NavSatFix,
            topic="/sensors/gps",
            callback=self.gps_callback,
            qos_profile=SENSORS_QOS_PROFILE,
        )

        # make a client to get target gnss coord in relation to map
        self._gps_to_map_client = self.create_client(GnssToMap, "convert_gps_to_map")

    # all ROS 2 nodes must be hashable!
    @override
    def __hash__(self) -> int:
        return super().__hash__()

    # Function to send lights request given a LightsRequest class instance
    def send_lights_request(self, lights_info: LightsRequest) -> LightsResponse | None:
        """
        Send a request to the lights service to turn the lights red.
        """
        lights_future = self._lights_client.call_async(lights_info)
        rclpy.spin_until_future_complete(self, lights_future)
        return lights_future.result()

    async def navigator(self):
        """
        This is an async function driven by the `asyncio` runtime created in
        the `main()` function at the bottom of this file.

        It's async such that any async functions we `await` in here have access
        to the runtime. (Otherwise, they wouldn't be able to run async.)

        Note that this function will stop running after it reaches its goal.
        """

        await self._wait_for_sensors()

        # step 1: nav to coordinate.
        #
        # no matter what, we're going to navigate to a coordinate, as our
        # other tasks also require going to one.
        #
        # ...grab the coordinate we want to head to
        target_coord: GeoPoint = self.nav_parameters.coord
        llogger.info(f"Step 1: go to coordinate! target: {target_coord}")

        dist_to_target_coord_m: float = dist_m_between_coords(
            self._last_known_rover_coord.position,  # pyright: ignore[reportOptionalMemberAccess]
            target_coord,
        )

        # if we're not at the coordinate, go there!
        if dist_to_target_coord_m > MIN_GPS_DISTANCE:
            llogger.debug(
                f"Not yet at target coordinate! Navigating... (coord: {target_coord})"
            )
            await self._go_to_coordinate(target_coord)

        # step 2: handle aruco/object detection, if we need to
        match self.nav_parameters.mode:
            case NavigationMode.GPS:
                pass
            case NavigationMode.ARUCO:
                _ = self.get_logger().info(
                    "At requested coordinate! Now searching for ArUco marker."
                )
                await self.handle_aruco_navigation()
            case NavigationMode.OBJECT_DETECTION:
                _ = self.get_logger().info(
                    "At requested coordinate! Now performing object detection search."
                )
                _ = self.get_logger().fatal("Object detection is unimplemented.")
                sys.exit(1)  # because it's unimplemented.
                # TODO(bray): remove the above.

        # step 3: we've reached our target! flash lights and return...
        _ = self.get_logger().info("Goal reached!")
        self._flash_lights()
        shutdown(self)
        return

    def stop_wheels(self):
        """
        Publish wheel speeds to stop the Rover.

        Note that Nav2 does this automatically - this is just a sanity check
        and safety measure.
        """
        # an empty twist message will stop the wheels
        zero_speed_twist: Twist = Twist()

        # publish it!
        self._cmd_vel_publisher.publish(zero_speed_twist)
        _ = self.get_logger().info("Wheels stopped!")

    async def _go_to_coordinate(
        self,
        dest_coord: GeoPoint,
    ):
        """
        Given a GPS coordinate, navigate to it.

        This async coroutine will monitor the status of the navigation and
        handle any nonsense that might pop up. Returns when completed.
        """
        # call into our `utm_conversion_node` and ask for a converted coordinate
        llogger.debug("creating utm request...")
        request = GnssToMap.Request()
        request.gnss_coord_to_convert = GeoPoint()
        request.gnss_coord_to_convert.latitude = dest_coord.latitude
        request.gnss_coord_to_convert.longitude = dest_coord.longitude
        llogger.debug("utm request created.")

        llogger.info("Waiting for UTM service to be ready...")
        _ = self._gps_to_map_client.wait_for_service()
        llogger.info("It's ready!")

        # get map coordinates from conversion service.
        #
        # we'll loop until we have a good one...
        point: Point
        while True:
            # call into the service and wait for it to complete
            resp_future = self._gps_to_map_client.call_async(request)
            while not resp_future.done():
                await asyncio.sleep(0.01)

            # ensure we got a response at all
            resp: GnssToMap_Response | None = resp_future.result()
            if resp is None:
                llogger.error("Failed to call service: got `None`. Retrying...")
                await asyncio.sleep(0.25)
                continue

            # check if we got it successfully
            llogger.debug("checking for response success...")
            if not resp.success:
                llogger.error("Service reported unhelp resp - will check again.")
                await asyncio.sleep(0.25)
                continue
            else:
                # break the loop if we got it!
                point = resp.point
                llogger.debug(f"got a good point! using this: {point}")
                break

        # translate our GeoPoint into local units. this is required for Nav2's
        # `planner_server` to accept our goal.
        #
        # (if it's the wrong type, it'll silently ignore u. fair warning lmao)
        llogger.debug("converting coordinate...")
        dest: PoseStamped = geopoint_to_pose(
            point,
            self.get_clock().now().to_msg(),
        )
        llogger.debug(f"coordinate converted! got: {str(dest)}")

        # create a `BasicNavigator` from `nav2_simple_commander`
        llogger.debug("creating basic navigator...")
        basic_nav: BasicNavigator = BasicNavigator(
            node_name="navigator_node_basic_navigator"
        )
        llogger.debug("basic navigator created!")

        # wait for Nav2 to be up...
        llogger.info("Waiting for Nav2 to be ready...")
        basic_nav._waitForNodeToActivate(node_name="bt_navigator")  # pyright: ignore[reportPrivateUsage]
        llogger.info("Nav2 appears to be ready! Continuing...")

        # start navigating.
        #
        # note that the `go_to_pose` function returns a bool indicating whether
        # the destination was "accepted" by Nav2.
        #
        # if it wasn't accepted, we'll wait a bit and try again.
        while True:
            if basic_nav.goToPose(dest):
                # move on to managing the basic nav
                break
            else:
                # wait a bit to try again
                llogger.error(
                    "Failed to navigate to destination: Nav2 rejected the target. Retrying in 5 seconds..."
                )
                await asyncio.sleep(5.0)
            pass
        pass

        # great, we've started navigating!
        #
        # let's sit here and manage it until it's done.
        while True:
            # when we've completed the navigation task, break out
            if basic_nav.isTaskComplete():
                break
            pass

            # otherwise, we're still navigating. wait a moment to check again
            await asyncio.sleep(1.0)
        pass

        # alright, we've finished coordinate navigation! let's tell the user...
        llogger.info("Navigated to target successfully!")

    # TODO(log): implement
    async def handle_aruco_navigation(self):
        """
        Performs the second step of the Navigator node (if requested), which is
        ArUco navigation.
        """

        goal_msg = FindArucoWithPose.Goal()

        # Wait for the ArUco action server to be up
        llogger.debug("Waiting for ArUco action server to be ready...")
        _ = self._aruco_client.wait_for_server()

        # Send empty goal to start the action
        llogger.debug("Sending goal to ArUco action server...")
        send_goal_future = self._aruco_client.send_goal_async(goal_msg, feedback_callback=self.aruco_feedback_callback)

        # Start async search task
        search_task = self.search_for_aruco()

        # Ensure we've at least seen the aruco marker once
        if self._curr_marker_transform is None:
            llogger.debug(
                f"Haven't seen the ArUco marker (id: {self._given_aruco_marker_id}) yet. Performing search algorithm..."
            )
            _ = self.get_logger().fatal("searching for aruco is unimplemented!")
            shutdown(self)
            sys.exit(1)
    
    async def search_for_aruco(self):
        """
        Moves the Rover circles until the target marker id is found.
        """
        # Create a priority queue of coordinates, where priority is distance to current location
        # OR most urgent priority for an estimated marker location
        coordinate_queue:PriorityQueue[tuple[float, GeoPoint]] = PriorityQueue()
        marker_missed_count = 0 # how many times have we been tracking and not seen the marker?
        tracking_marker = False # are we currently tracking a marker?
        last_received_feedback_time: Time | None = None

        # Variable to store the current search task, so we can cancel it if we lose the marker
        current_search_task: asyncio.Task | None = None

        while True:
            # Check if new feedback received. If not, wait and check again
            if self._aruco_action_feedback is None or self._aruco_action_feedback.time_last_image_arrived == last_received_feedback_time:
                llogger.info("Waiting for feedback from ArUco action server...")
                await asyncio.sleep(0.5)
                continue

            # Check if coordinate of rover is known. If not, wait and check again
            if self._last_known_rover_coord is None:
                llogger.info("Waiting for GPS coordinate to be known...")
                await asyncio.sleep(0.5)
                continue

            # Store time of last received feedback
            last_received_feedback_time = self._aruco_action_feedback.time_last_image_arrived
            
            # Check if marker is found in list of marker ids
            if self._given_aruco_marker_id in self._aruco_action_feedback.marker_ids:
                llogger.info("Marker seen!")
                marker_index = self._aruco_action_feedback.marker_ids.index(self._given_aruco_marker_id)

                if not tracking_marker:
                    tracking_marker = True
                    marker_missed_count = 0
                    # Convert marker pose to coordinate and add to priority queue with high priority
                    self._last_known_marker_coord = coordinate_from_aruco_pose(
                        self._last_known_rover_coord,
                        self._aruco_action_feedback.marker_poses[marker_index],
                    )
                    coordinate_queue.put((0.0, self._last_known_marker_coord.position))

                    # Go to the marker coordinate
                    # If search task, cancel it and start new one to go to marker coordinate
                    if current_search_task is not None:
                        current_search_task.cancel()
                    current_search_task = asyncio.create_task(self._go_to_coordinate(self._last_known_marker_coord.position))
                else:
                    # If we're already tracking the marker, check distance to coordinate
                    distance_to_marker = get_distance_to_marker(self._aruco_action_feedback.marker_poses[marker_index])

                    if distance_to_marker <= MIN_ARUCO_DISTANCE:
                        llogger.info("Reached marker!")
                        self.stop_wheels()
                        return

            elif tracking_marker:
                # if we were tracking the marker but now it's not seen, increment missed count
                marker_missed_count += 1
                # if missed count is too high, cancel current search task and stop tracking
                if marker_missed_count > 20: # TODO: add parameter for missed count threshold
                    llogger.info("Marker lost. Stopping tracking and resuming search...")
                    if current_search_task is not None:
                        current_search_task.cancel()
                    tracking_marker = False
                # Add some time delay before checking for marker again to avoid spamming the action server with requests
                await asyncio.sleep(0.5)

            else:
                # Check if queue empty
                if coordinate_queue.empty():
                    # Generate similar coordinates to current location and add to queue with priority based on distance to current location
                    similar_coords = generate_similar_coordinates(self._last_known_rover_coord.position, radius=10.0, num_points=10) # TODO: add parameters for radius and number of points
                    for coord in similar_coords:
                        distance_to_coord = dist_m_between_coords(self._last_known_rover_coord.position, coord)
                        coordinate_queue.put((distance_to_coord, coord))
                else:
                    # Get next coordinate from queue and go to it
                    next_coord = coordinate_queue.get()[1]
                    llogger.info(f"Going to next search coordinate: {next_coord}")
                    current_search_task = asyncio.create_task(self._go_to_coordinate(next_coord))

    def _flash_lights(self):
        """
        we have reached our goal, so we are going to ask the `lights_node` to
        flash the lights for us, then shut down.

        flashing the lights is a requirement from
        """
        # Set lights to FLASHING GREEN
        lights_info: LightsRequest = LightsRequest()
        lights_info.red = 0
        lights_info.green = 255
        lights_info.blue = 0
        lights_info.flashing = True
        _ = self.send_lights_request(lights_info)
        return

    async def _wait_for_sensors(self):
        """
        This function will return after we get sensor information from all
        required sensors.

        Await it to idle until we have sensor data.
        """
        # Ensure we've started receving rover coordinates
        while self._last_known_rover_coord is None:
            llogger.warning(
                "Can't start navigating until the GPS provides a coordinate. Waiting to begin..."
            )
            await asyncio.sleep(0.5)
        pass

    def gps_callback(self, msg: NavSatFix):
        # move its header over
        gp_stamped: GeoPointStamped = GeoPointStamped()
        gp_stamped.header = msg.header

        # and stick the coord stuff in a geopoint
        gp: GeoPoint = GeoPoint()
        gp.latitude = msg.latitude
        gp.longitude = msg.longitude
        gp.altitude = msg.altitude

        # mv the geopoint into that parent type
        gp_stamped.position = gp

        # finally, set that type on the navigator class
        self._last_known_rover_coord = gp_stamped

    def aruco_feedback_callback(self, feedback: FindArucoWithPose.Feedback):
        # Store feedback
        self._aruco_action_feedback = feedback.feedback

        # Log feedback (can remove later if this clogs our logs)
        llogger.debug(f"Received feedback from ArUco action server: {feedback}")


"""
HEY!

The following is related to starting the `asyncio` runtime and creating tasks
to handle the ROS 2 node + `rclpy` connection.

You don't need to touch it when adjusting behavior...
"""


def main(args: list[str] | None = None):
    """
    Starts the Navigator node.
    """
    rclpy.init(args=args)
    navigator_node: NavigatorNode = NavigatorNode()

    # spin the ros 2 node and run our logic... at the same time!
    #
    # for more info, see: https://github.com/m2-farzan/ros2-asyncio
    future = asyncio.wait(
        [
            asyncio.create_task(ros_spin(navigator_node)),
            asyncio.create_task(navigator_node.navigator()),
        ]
    )
    _ = asyncio.get_event_loop().run_until_complete(future)

    # destroy the Node explicitly
    #
    # this is optional - otherwise, the garbage collector does it automatically
    # when it runs.
    navigator_node.destroy_node()
    rclpy.shutdown()


def shutdown(n: NavigatorNode):
    """Immediately turns off the Navigator."""
    n.stop_wheels()
    rclpy.shutdown()


async def ros_spin(node: Node):
    """
    Spins the given ROS 2 node. This allows it to connect to other DDS
    entities.
    """

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-4)

    llogger.error("`rclpy.ok()` no longer True. the node is no longer spinning.")
