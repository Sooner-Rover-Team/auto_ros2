# Notes on how we detect ArUco markers

## `aruco::aruco_node`

The `aruco_node` starts with `drive.launch.py`, mostly existing as an action server. When it has at least one running action, it starts processing `/sensors/mono_image` for ArUco markers. (in other words, it's explicitly **not processing** when there are no action instances)

Each action can technically run forever, though it'll stop one of two ways:

1. The client (`navigator_node`) stops caring and either shuts down, or otherwise disconnects from the action server (`aruco_node`).
2. `navigator_node` tells the action server (`aruco_node`) to stop running the action, which always succeeds.

## `navigator::navigator_node`

This node actively tries to find a specific ArUco marker (given by URC staff).

### Parameters

When the navigator is first started, we give it a number of parameters:

- `aruco_marker_id: int` is the ArUco marker the Navigator tries to visit, stopping at a distance of 2 meters or less.
- `coord: GeoPoint` is a GPS coordinate where the Navigator will visit. In ArUco mode, that means we'll go to this coordinate while trying to find the marker, but if we don't find it, we'll employ a search algorithm (since the marker is within `n` meters of the given coordinate).

### Behavior

Since we're trying to find an ArUco marker near a coordinate, we'll just immediately start the `aruco::aruco_node` with `aruco_marker_id` while navigating to the target coordinate.

During the coordinate navigation, we'll constantly watch for feedback from the `aruco_node`. That'll say the last time we processed an image and give us info about any ArUco markers we found (alongside their poses).

Any time we find the marker, we convert its Rover-local pose to UTM and save it in the `NavigatorNode` class.

When we've reached the coordinate, we'll react to one of two possibilities:

1. We already found a marker! Drive to its saved UTM coordinate.
  - If we don't end up at the marker when we arrive, we'll start a search algorithm near the UTM coordinate instead.
2. No marker found
  - let's perform a search algorithm. When we find something, navigate to its coordinate.
