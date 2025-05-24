# Sensors

On the Rover, we have a lotta sensors to maintain Autonomous operations.

## GPS

- connection: TCP (through the router. see [`network.md`](../network.md) for more info!)

Our GPS is a Swift Navigation Piksi Multi. It provides a multitude of measurements, some of which only appear when [setting up the Base Station](https://support.swiftnav.com/support/solutions/articles/44001904334-piksi-multi-gnss-rtk-position-with-stationary-base). Having the Base Station can also increase the reliability of data. We're not currently using this enhanced data, but doing so is absolutely possible (and preferable).

## Color Camera

- connection: USB 3.1

For our color camera, we use the [See3CAM_24CUG](https://www.e-consystems.com/industrial-cameras/ar0234-usb3-global-shutter-camera.asp). It seems like a good fit due to its decent resolution, high refresh rate, lossless quality, and global shutter support. The manufacturer, e-con Systems, previously developed a [proof-of-concept license plate identification](https://www.youtube.com/watch?v=nCaN9LarqSA) system using this camera, so its range seems to be acceptable for ArUco marker detection at a decent distance.

However, its a fixed focus camera, and no approximation of its range was given in its specification sheet. Please be wary of this situation during testing and development, and don't be afraid to question the hardware to look into a new lens.

### Mount

The camera is mounted 0.5782 meters above the bottom of the ebox.

## Depth Camera

- connection: USB 3.1
- datasheet: [available here](https://cdn.sanity.io/files/s18ewfw4/staging/c059860f8fe49f3856f6b8da770eb13cc543ac2c.pdf/ZED%202i%20Datasheet%20v1.2.pdf)

We use a [ZED 2i](https://www.stereolabs.com/store/products/zed-2i) depth camera for our object avoidance and real-time mapping.

## IMU (including Compass)

- connection: UDP (info provided by Electrical through their microcontrollers. see [`network.md`](../network.md)...)

To better understand the heading of the Rover, we use an IMU (the [ICM-20948](https://www.adafruit.com/product/4554)). We already have another source of heading information from the Piksi, so it should be possible to improve tracking in the future.

## Future Sensors

We don't currently have LiDAR technology on the Rover, but we'd like to have it eventually. Other sensors are also on the table!
