# RHR-setup-ROS
Setup of RHR in ROS

## Installation

Make catkin workspace:

    mkdir -p ~/catkin_ws/src
    
Clone repository:

    cd ~/catkin_ws/src 
    git clone https://github.com/SINTEF-INRIA/RHR-setup-ROS

Update submodules:

    cd RHR-setup-ROS
    git submodule init && git submodule update

Catkin make (Installs pkg)

    cd ~/catkin_ws/
    catkin_make

First, you need to get the cross-compiler (gcc-arm) and build OpenOCD, the JTAG transport, and the DFU (Device Firmware Updater) utility. On Ubuntu 12.04 and 14.04 (and potentially others), this is all scripted for you:

    cd ~/catkin_ws/src/RHR-setup-ROS/Installation/reflex-ros-pkg
    cd firmware/tools 
    make 
    make dfu

Now you can build the firmware image. If you are using the ReFlex Takktile, 

    cd firmware/reflex-takktile
    make clean
    make

