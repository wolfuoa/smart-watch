# COMPSYS 704 project 2 (Group 3)
Implementing al-time orientation and step detection without relying on GPS, programmed on the SensorTile board with an embedded STM32 chip.

# About
This repository documents the development process of an embedded pedestrian dead-reckoning system. The device uses a 3D accelerometer and magnetometer to infer
the user's steps, heading, pace, and distance traveled from the origin. 

# Code Structure
- doc: Contains relevant datasheets for MCUs and ASICs used in this project
- src/COMSYS704/Projects/STM32L476JG-SensorTile/Applications: Contains subfolders for source code
    - inc: Contains all include files
    - src: Contains all source files

## Nagivating the Project
As per the code structure mentioned above, navigate to the `/inc` and `/src` directories.

Within these directories are all files modified or created by Group 3:
- main.c/h
- accelerometer.c/h
- magnetometer.c/h
- gyroscope.c/h
- filter.c/h
- step_metrics.c/h

Please see below for explanation on all files
- `main.c`: Contains all control and intialization code as well as distance tracking and data broadcasting
- `accelerometer.c/h`: Contains abstraction for using the accelerometer
- `magnetometer.c/h`: Contains abstraction for using the magnetometer
- `gyroscope.c/h`: Contains abstraction for using the gyroscope
- `filter.c/h`: Implements a modular moving average window filter for integer-type datas
- `step_metrics.c/h`: Implements the step detection algorithm and communicates to main if a step is detected

# Running The Code
To build and flash this code onto the SensorTile, perform the following steps:
1. Clone the repository or download the source code on a Windows 10/11 machine that has the STM32CubeIDE installed.
2. Open STM32CubeIDE and click `file > import > General > Existing Projects into Workspace > Next > Browse`, then navigate to `~/smart-watch` then click `finish`. 
3. In the projects window, ensure that only one tickbox is selected, named `COMPSYS704 (~/smart-watch/src/COMPSYS704/Projects/STM32L476G-SensorTile/Applications/ALLMEMS1/STM32CubeIDE)`.
4. Along the top bar click on the hammer icon to build the project.
5. Once the project has finished building, click on the green play button also along the top bar, to run and flash the code to the SensorTile.
6. The performance can be validated via putty, or by connecting to the SensorTile via a mobile app such as "ST BLE Sensor".
7. On the BLE Sensor app, click on Plot Data and select one of three tabs: Accelerometer, Gyroscope, Magnetometer
8. Press the play button to receive data. Magnetometer displays the raw sensor data, Accelerometer displays averaged acceleration data, and Gyroscope displays total steps, heading and distance traveled.


