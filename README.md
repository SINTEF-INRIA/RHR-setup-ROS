# RHR-setup-ROS
Setup of RHR in ROS

## Installation

Clone Repository:

```git clone https://github.com/SINTEF-INRIA/RHR-setup-ROS```

Change directory:

```cd RHR-setup-ROS```

Pull submodules:

```git submodule init && git submodule update```

Change directory:

```cd Installation/reflex-ros-pkg```

### Flashing firmware
------
First, you need to get the cross-compiler (gcc-arm) and build OpenOCD, the JTAG transport, and the DFU (Device Firmware Updater) utility. On Ubuntu 12.04 and 14.04 (and potentially others), this is all scripted for you:

    cd firmware/tools 
    make 
    make dfu

Now you can build the firmware image. If you are using the ReFlex Takktile, 

    cd firmware/reflex-takktile
    make clean
    make
