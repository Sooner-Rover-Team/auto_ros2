# ARCHIVED: Our previous design for the `navigator` package

This file contains our previous design for the `navigator::navigator_node`. It was created before we switched over to Nav2, a library that handles navigating for us.

However, we keep it in the repo for future reference. (i.e., we could switch the design again later)

## Wheel speeds between Rover and Target

In most robotics projects, you'll use a few components to calculate the wheel speeds (to move):

- Rover location
  - We get the Rover's coordinate location from the GPS a number of times per second.
  - In our program, we'll use that to determine the difference between where we're at, and where the target coordinate is at.
- Target location
  - Either:
    - we give the Navigator a target location, or
    - it'll have to calculate the location of the target.
  - Once Nav knows where the target is, we calculate the coordinate difference.
- PID controller: makes the Rover less "jumpy"

### Calculation

So, with those variables, we can do a little calculation:

**1. Grab coordinate difference**

```python3
dx: float = target_coord.x - rover_coord.x
dy: float = target_coord.y - rover_coord.y
```

**2. Complete angle calculation**
  - note: this uses the [atan2 (link: Wikipedia)](https://en.wikipedia.org/wiki/Atan2) mathematical func.
  - ...which means we just grab the angle for free. See that link for more info.

```python3
angle_to_target: float = atan2(dy, dx)
```

**3: Find "error"**
  - defined as the target's *heading* (facing angle/direction) compared to Rover's heading
    - compass is required for trustworthy results. always have a compass (magnetometer) on the Rover.
  - (in other words, error is just how much we need to change Rover's heading)

```python3
error: float = angle_to_target - rover_compass_value
```

**4: Stick error in PID controller**
  - in our codebase, we just use a library for this.

```python3
# see: https://simple-pid.readthedocs.io/en/latest/
from simple_pid import PID as Pid

output_t = Pid(error)
```

**5: Send that PID output to wheels**
  - it's a correction value, and we're using (in principle, not necessarily mechanically) [differential drive](https://en.wikipedia.org/wiki/Differential_(mechanical_device)).
  - doesn't matter which side is neg, which is pos: just be consistent!
  - we can define `base_speed` as `i8::MAX - (i8::MAX / 3)`, or 2/3 max speed.
    - also note that this is arbitrary; we can play around with it.

left_wheel_speed = base_speed - correction
right_wheel_speed = base_speed + correction

## Calculate an ArUco marker's coordinate

Let's review the data we have:

- Camera calibration parameters. Follow the steps here to run the [`cv::calibrateCamera`](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html#autotoc_md1194) function. Save the output. Re-run if the camera's settings are changed.
  - It's good practice to re-check this every so often.
  - [This article](https://web.archive.org/web/20250116052710/https://automaticaddison.com/how-to-perform-pose-estimation-using-an-aruco-marker/) has a nice guide on the process.
- Rover coordinate location, from GPS.
- ArUco marker, placed in real life (or in simulator).

And, yeah, that's all you need. Let's follow some steps to grab the coordinate:

**1: "Solve" for 3D marker pose (rot, pos) compared to camera**
  - in short, we're just comparing the marker's location to the camera, with the camera acting as the origin.

Use [`cv::SolvePnP()`](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d) to get `rvecs`, `tvecs` vectors.

We're only looking for one ArUco marker, so just grab the first element from both vectors.

(so, `rvec: RotationVector`, `tvec: TranslationVector`)

**2: Put pose onto GPS coordinate space**
  - we're working on "Mars" (Earth), so the camera isn't actually the origin.
  - so, instead of the camera, we'll move the pose (`(rvec, tvec)` pair) we found to the global coordinate system
  - note: based off [a Python example](https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/aruco_pose_estimation.py) in the *How do drones work* series by profressional aero engineer Tiziano Fiorenzani.
  - also, we might need to flip the matrices [because of something like this](https://www.politesi.polimi.it/retrieve/28ae3d35-c582-4bb8-887c-79d7c531cb68/2022_12_Jeyakodi_David_Jim_Fletcher.pdf#page=19)
  - also also, the comments on [this stack overflow question](https://stackoverflow.com/questions/66930170) show why we used a compass + gps for this calculation lol

```rust
use cv2::Mat;

// "eye" method makes an identity matrix
let mut camera_pose_matrix: Mat = Mat::eye(4, 4);
let rover_pose_matrix: Mat = Mat::eye(4, 4);

// copy values to imatr
rvec.copy_to(camera_pose_matrix.select(cv2::Rect(0, 0, 3, 3)));
tvec.copy_to(camera_pose_matrix.select(cv2::Rect(3, 0, 1, 3)));

// multiply the two matrices together.
//
// note: since `robot_pose_matrix` is an identity matrix, this is a no-op.
//       however, if you want to specify that the camera isn't *perfectly*
//       in line with the Rover's compass, or that it's noticably shifted from
//       the GPS, you can modify the pose matrix to say so.
let adjusted_marker_pose: Mat = camera_pose_matrix * rover_pose_matrix;
```

This outputs a matrix containing some rotation stuff we don't really use right now, alongside an important translation vector with `(x, y, z): (f64, f64, f64)` offsets in meters.

**3: Make an actual coordinate**
  - `geographiclib` carries here

```python3
"""
note: technique somewhat inspired by this C++ project:
https://github.com/Zachamus/Raytheon_Drone_Comp/blob/d5717d38d/Raytheon_Capstone/SearchAlgo.cpp#L45
"""

from math import sqrt
from geographiclib.geodesic import Direct as direct

# assume `adjusted_marker_pose` is what we got from step 2.
#
# note: this example "array" came from github copilot, so please keep your
# expectations low lol
adjusted_marker_pose = np.array([
    [r11, r12, r13, t_x],
    [r21, r22, r23, t_y],
    [r31, r32, r33, t_z],
    [0,   0,   0,   1]
])

# grab (lat, lon, altitude) m offsets as t(ranslation)_{x, y, z}.
#
# warning: this assumes a correct mapping between coordinate spaces! which
# might be inaccurate. check this too...
lat_offset_m: float = adjusted_marker_pose[0][3]
lon_offset_m: float = adjusted_marker_pose[1][3]
alt_offset_m: float = adjusted_marker_pose[2][3]

# grab distance in meters using lat, lon offsets
dist_m: float = sqrt(lat_offset_m**2 + lon_offset_m**2)

# and angle to the marker
angle_rad: float = math.atan2(lat_offset_m, lon_offset_m)

# then use these in a coordinate offset calculation.
#
# rover_pos is used as `coord1` in this calculation:
# https://geographiclib.sourceforge.io/html/python/code.html#geographiclib.geodesic.Geodesic.Direct
output: dict[float] = direct(rover_pos.lat, rover_pos.lon, math.degrees(angle_rad), dist_m)

# unwrap these from the big-ahh dict it returns for some reason.
#
# we want `coord2` from that:
# https://geographiclib.sourceforge.io/html/python/interface.html#dict
aruco_lat: float = output["lat2"]
aruco_lon: float = output["lon2"]
```
