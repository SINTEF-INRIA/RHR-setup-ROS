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

Since we are using Ubuntu 16.04, there's been some problems. In the Makefile within ```firmware/tools``` we've change one line from referencing a repository with cannot access to one with access. (However, with 18.04 we needed to target the artful repository as bionic is non-existent at the moment. You can re-target with ```sudo nano /etc/apt/sources.list.d/tema-gcc-arm-embedded-ubuntu-ppa-`lsb_release --codename | cut -f2`.list```. Replace releasename with a viable one from Team-GCC-arm-embedded. Then update: ```sudo apt update```.)

    cd ~/catkin_ws/src/RHR-setup-ROS/Installation/reflex-ros-pkg
    cd firmware/tools 
    make 
    make dfu

Now you can build the firmware image. If you are using the ReFlex Takktile, 

    cd firmware/reflex-takktile
    make clean
    make

