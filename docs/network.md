# Network

Info about how the network is set up.

## Router

We use an Netgear Nighthawk AX1800 WiFi Router (R6700AX) on firmware version v1.0.11.112. For info about the long-distance networking setup, see [`docs/hardware/base_station.md`](hardware/base_station.md).

## Jetson Nano Orin (Autonomous - Computer)

- MAC Addr: `48:b0:2d:eb:f2:d9`
- Static IP: `192.168.1.68`

## Teensy (Electrical - Microcontroller)

- MAC Addr: `ae:ae:bc:9b:fb:21`
- Static IP: `192.168.1.102`
- Ports: these are the different ports to control various parts of the Rover.
  - wheels: `5002`
  - lights: `5003`
  - arm: `5004`
  - science: `5005`
  - imu: `5006`
    - 9-DoF sensor package - [ICM-20948](https://www.adafruit.com/product/4554)
  - battery: `5007`
    - indicates how charged the batteries are
