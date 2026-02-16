# `soro_gps`

Connects to the GPS and provides its data in a readable format.

## Testing

To test the GPS connection, use the following command:

`timeout 3s sh -c 'dd if=/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3844945-if02 bs=1 count=64 status=none | xxd -g 1 -u'`

If that works, great! Otherwise, you'll want to find the device path: `ls /dev/serial/by-id/ | grep Septentrio`. Then, replace the path in the command above with the one you got from that command. If there's multiple, test them all.

On the other hand, if none of those work, you should try configuring the streams on the GPS receiver. Ensure it's plugged into a computer with a display (like a laptop), then navigate to [its configuration page](http://192.168.3.1) and select the "NMEA/SBF Out" tab.
