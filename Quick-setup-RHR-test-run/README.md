# Quick setup & Test running RHR

This will go through some test-setups of the hand, how to initialize, how to move the fingers. More documentation can be found at: https://www.labs.righthandrobotics.com/docs (LabSINTEF, norm).

## Installation

Follow: ```SINTEF-INRIA/RHR-setup-ROS/Installation/README.md```

## Before running

Check that all necessary packages are available, ```cd ~/catkin_ws/ catkin_make``` should return message showing all these packages:

    rospack find reflex_driver
    rospack find reflex_msgs
    rospack find reflex

## ALX code example

This is the code that was used prior. However, we might need to do it where differently. In our trails we would most likly want the code to be: get.values_for_takketile_sensors, && move.finger[x].

I'll (AXL) look for these functions (20180525-1550)
