# Firmware for a SnapperGPS with accelerometer daughter-board

*Author: Jonas Beuchert*

The daughter-board is built around a [LIS3DH 3-axis MEMS accelerometer](https://www.st.com/en/mems-and-sensors/lis3dh.html).
It is soldered onto the five pads of the SnapperGPS main board.
The daughter-board is orientated such that the two pads labeled `VDD` connect.
The two boards communicate via I2C.

The z-axis of the accelerometer points downwards, i.e., away from the antenna.
The x-axis points in the same direction as the USB connector.
The y-axis points away from the side of the board with the pads.

To use the accelerometer, flash [this firmware](https://github.com/SnapperGPS/snappergps-firmware/releases/download/accelerometer-v0.0.1/acceleration-snapper.bin) using [https://snappergps.info/flash](https://snappergps.info/flash).

You can configure the SnapperGPS as normal.
In addition to GPS snapshots, it will then also capture accelerations in a rate that is twenty times higher than the configured GPS snapshot rate.

The accelerations are read out via [https://snappergps.info/accelerometer](https://snappergps.info/accelerometer).
The data is returned as JSON file and as CSV file, both of which contain the same data.
The measurements are in units of `g` (gravitational acceleration).

A simple Python script to plot accelerations can be found [here](https://github.com/snapperGPS/snappergps-scripts).
