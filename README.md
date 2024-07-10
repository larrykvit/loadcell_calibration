# Loadcell calibration

Calibrate a mavin NA128 loadcell by using a certified calibrated loadcell.

Control & data collection for the DIY loadcell calibration jig.
Processing of the data to create a calibration value.

## Hardware layout

8020 aluminum frame with linear rails for the structure.

Vevor 12V linear motor for motion. It is a brushed motor internally, so it has 2 wires for DC control. It has limit switches that shut it off when it reached either end of motion.

The 12v linear motor is controlled by a [`MCP236`](https://www.basicmicro.com/MCP236-Dual-30A-60VDC-Advanced-Motor-Controller_p_31.html) motor controller.

The motor controller is connected to a PC via a USB cable.

The calibrated loadcell is [`Omega LC103B-500`](https://www.omega.ca/fr/force-and-strain-measurement/load-cells/lc103b/p/LC103B-500).

Both calibrated and uncalibrated loadcells are connected to strain gauge amplifier & ADC.

Currently that is a [`Phidget Bridge 4 input 1046_1`](https://www.phidgets.com/?prodid=1270). But might be changed once custom PCB is done.

## Interface

### Motor controller

The MCP motor controller has the same API as the roboclaw motor controllers. They have a python implementation on their [website](https://www.basicmicro.com/motor-controller-downloads). That file is included in this repo.

The MCP motor controller requires `PySerial`: `pip install pyserial`.

### Loadcell

The phidgets bridge control also has a python library.

Two options for installation:
- Install the [C library](https://www.phidgets.com/docs/Language_-_Python) & python wrappings separately `pip install Phidget22`
- Install the python library with the embedded C library `pip install Phidget22Native`

I don't know which is better.

[Python API documentation](https://www.phidgets.com/?view=api&product_id=1046_1&lang=Python)

