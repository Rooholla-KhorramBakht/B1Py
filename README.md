# B1Py
This repository hosts tools, documentations, example controllers, and state estimators for the Unitree B1 robot. Rather than focusing on a cutting-edge locomotion stack, this repository aims to bring simplicity by providing modular and reusable building blocks developed in Python. However, we do not stop here. Python Interfaces to advanced state estimators and controllers are also available for more serious use cases. 

![](docs/images/open_figure.png)

<!-- The communication to the robot is made possible through the 'unitree_legged_sdk' and is carried out over the UDP link to the robot's onboard computer. High-level interface commands B1's onboard locomotion controller (running on the onboard intel computer with IP: `192.168.123.220`) and the low-level interface directly communicates with the onboard data acquisition micro controller (with IP: `192.168.123.110`). -->

## Installation
Simply install for your python interpreter of interest using pip:

```bash
git clone https://github.com/Rooholla-KhorramBakht/B1Py.git 
cd B1Py
<path/to/python> -m pip install .
```

## Getting Started
The following point to the documentations and Jupyter notebook examples for various use cases and procedures:
### Setup and Calibration
- [Network Setup and Internet Sharing]()
- [NTP Time Synchronization Between the Onboard Computers]()
- [Onboard Realsense D430i Setup]()
- [Extrinsic and Intrinsic Calibration]()

### Control
- [Joystick with the Unitree High-Level Controller](notebooks/unitree_highlevel_joystick_control.ipynb)
- [Position Control with Unitree High-Level Controller and Vicon](notebooks/unitree_highlevel_position_control_vicon.ipynb)
- [Position Control with Unitree High-Level Controller and VIO/LiDAR]()

