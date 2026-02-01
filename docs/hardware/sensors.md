# Sensors

On the Rover, we have a lotta sensors to maintain Autonomous operations.

## GPS

- connection: USB 2.0 (with a bundled driver that fakes a serial connection)

We use the [Septentrio mosaic-Go Heading](https://www.digikey.com/en/products/detail/septentrio-inc/410397/18091780) evaluation kit as the GNSS (GPS) receiver in our Rover. The device supports up to 2 antennas, though we only use one at the moment. The antenna we use is the Swift Navigation GPS500. It connects to the GNSS antenna over a regular old TNC connector. There are several of those cables scattered around the Bay; usually, one is connected to the antenna already.

The mosaic-Go plugs into any computer via a micro-USB cable; this both powers it and provides a web interface on a fake network connection. **You do not need any other cable to connect; just that one micro-USB**. To configure the mosaic-Go, just plug it into your computer, open your network settings, and find its IP. For me, it was accessible at [this IP: `192.168.3.1`](https://192.168.3.1/scr). Please back up the configuration before making any changes.

## Color Camera

- connection: USB 3.1

For our color camera, we use the [See3CAM_24CUG](https://www.e-consystems.com/industrial-cameras/ar0234-usb3-global-shutter-camera.asp). It seems like a good fit due to its decent resolution, high refresh rate, lossless quality, and global shutter support. The manufacturer, e-con Systems, previously developed a [proof-of-concept license plate identification](https://www.youtube.com/watch?v=nCaN9LarqSA) system using this camera, so its range seems to be acceptable for ArUco marker detection at a decent distance.

However, its a fixed focus camera, and no approximation of its range was given in its specification sheet. Please be wary of this situation during testing and development, and don't be afraid to question the hardware to look into a new lens.

### Mount

The camera is mounted 0.5782 meters above the bottom of the ebox.

## Depth Camera

- connection: USB 3.1
- datasheet: [available here](https://cdn.sanity.io/files/s18ewfw4/staging/c059860f8fe49f3856f6b8da770eb13cc543ac2c.pdf/ZED%202i%20Datasheet%20v1.2.pdf)
- formats
  - output: only supports `YUYV`

We use a [ZED 2i](https://www.stereolabs.com/store/products/zed-2i) depth camera for our object avoidance and real-time mapping.

## IMU (including Compass)

- connection: UDP (info provided by Electrical through their microcontrollers. see [`network.md`](../network.md)...)

To better understand the heading of the Rover, we use an IMU (the [ICM-20948](https://www.adafruit.com/product/4554)). We already have another source of heading information from the Piksi, so it should be possible to improve tracking in the future.

## Future Sensors

We don't currently have LiDAR technology on the Rover, but we'd like to have it eventually. Other sensors are also on the table!
