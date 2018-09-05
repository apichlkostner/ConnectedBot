# Connected Robot

Robot based on https://github.com/apichlkostner/robot

## HW nodes

* Desktop PC
* Raspberry Pi
* uC board

## Sensors

* Camera
* IR distance sensors
* motor encoders

## Connectivity

* ROS, ROSSerial
* LAN
* WLAN
* UART

## ROS nodes

* roslaunch script to launch all nodes on Desktop, Raspberry Pi and uC board
* Low level on uC board 
    * IR sensors
    * encoders
    * motor driver
* Application on Raspberry Pi
    * camera publisher node
    * joystick to motor driver
* Performance and user interaction on desktop
    * camera viewer
    * joystick driver node