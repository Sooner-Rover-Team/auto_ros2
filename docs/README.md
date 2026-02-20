# Documentation

Thanks for taking a look at the SoRo repo! This folder contains the project's documentation, including tutorials, an FAQ, and design documents. If you're confused about something, it's probably documented here.

Here's a short list describing the info that each file provides:

- [`base_station.md`](base_station.md): about setting up the "base station", our giant antenna for remote operations.
- `design/`: the design + structure of the project
  - [`design/tf2.md`](design/tf2.md): definitions for the coordinate space frames (transformations); how the ROS 2 `tf2` library is set up; a list of all frames + links
- [`faq.md`](faq.md): a giant list of bugs + footguns we've encountered. (very helpful for new members and the ROS 2 community as a whole)
- `hardware/`: about the Rover hardware
  - [`hardware/jetson.md`](hardware/jetson.md): introduction to the NVIDIA Jetson Orin Nano computer we use on the Rover
    - also includes setup advice, just in case the storage device breaks
  - [`hardware/sensors.md`](hardware/sensors.md): the sensors/inputs used on the Rover (like cameras, GPS, and IMU)
- [`nav.md`](nav.md): description of our design for navigation, pre-Nav2.
  - WARNING: These docs are no longer immediately relevent; our design needs changed with the switch to Nav2. Nonetheless, we keep these ideas present in case we refactor to have additional control.
- [`network.md`](network.md): static IP + MAC address assignments (i.e., how to connect to X or Y devices)
- [`offline.md`](offline.md): tutorial on how to update/modify the Rover code and its dependencies (even if we have NO way to get an internet connection on the Rover)

## Adding Documentation

Did you figure something out the hard way? Please write it down somewhere in this folder, then add any new file(s) to the above list!

This project is intended to help people learn about robotics and computer science, so lots of documentation will go a long way. :)
