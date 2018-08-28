PLATFORM=CF2
#ESTIMATOR=kalman

DEBUG=0
## Automatically reboot to bootloader before flashing
CLOAD_CMDS = -w radio://0/100/2M/E7E7E7E7E4
##CLOAD_CMDS = -w usb://0

##Build for TDoA v3
#CFLAGS += -DLPS_TDOA_ENABLE -DLPS_TDOA_USE_V3



## Set number of anchor in LocoPositioningSystem
CFLAGS += -DLOCODECK_NR_OF_ANCHORS=8



## 1 enables, 0 disables, must make clean after change
AUTONOMOUS_FLY=1


## Compile positioning system for TDoA mode
LPS_TDOA_ENABLE=1
