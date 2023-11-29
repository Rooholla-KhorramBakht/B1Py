# System Setup

## Jetson Orin Auxiliary Computer  

We have installed an NVIDIA Jetson Orin as the main computer on the robot. First flash the board as explained [here](). Then move the root file system to the SSD (we assume that an SSD driver is installed) using the [rootOnNVMe tool](https://github.com/jetsonhacks/rootOnNVMe). Also make sure that the Jetpack is installed on the newly flashed image. Then follow through:

- [CUDA Libraries](orin/cuda.md)
- [Network Setup](orin/network.md)
- [Sensor Drivers](orin/sensor.md)
- [System Config](orin/autostart.md)