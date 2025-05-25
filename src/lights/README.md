# `lights`

Contains the `lights_node`.

## Usage

You can run it with: `ros2 run lights lights_node`

Testing is fairly simple. Run `ros2 service call /lights_service custom_interfaces/srv/Lights "{red: 255, green: 255, blue: 255, flashing: False}"` from any computer on the LAN.
