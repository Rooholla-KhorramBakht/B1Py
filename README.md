# B1Py
B1Py is a collection of tools and guides for getting started with the Unitree B1 robot. It aims to bring simplicity by providing a modular and as much as possible Pythonic toolchain for this robot. It features:

- High-level interface abstraction with safety features to communicate with the onboard controller.
- Low-level joint interface abstraction with safety features to control the robot's actuators.
- A set of docker files to automate the launch of all the sensors and interfaces of the robot on startup.
- A set of tutorials for calibrating the sensors and running basic computational components such as VIO, elevation mapping, etc...

<!-- ![](docs/images/open_figure.png) -->

<!-- The communication to the robot is made possible through the 'unitree_legged_sdk' and is carried out over the UDP link to the robot's onboard computer. High-level interface commands B1's onboard locomotion controller (running on the onboard intel computer with IP: `192.168.123.220`) and the low-level interface directly communicates with the onboard data acquisition micro controller (with IP: `192.168.123.110`). -->

## How to Use
### Simulation
TODO
### Hardware
Follow through the following steps to setup and calibrate the robot.
#### Setup
- [Operation Basics]()
- [System Setup](docs/setup/index.md)
- [Vicon Setup](docs/setup/vicon.md)
- [Python Interface](notebooks/unitree_locomotion_controller_interface.ipynb)

#### Calibration
- [Intrinsic Calibration]()
- [Extrinsic Calibration]()
- [Robot Description and `B1Params` Class]()

#### Basic Nodes
- [Robot-Centric Elevation Mapping]()
- [Visual-Inertial-Legged Odometry]()
- [LiDAR Odometry]()

Now that the robot is operational, we can use installed nodes and Python classes to communicate with the robot and use it in research. 

### Tutorials
Following is a list of tutorials to get your hands dirty: 

#### Control Through High-Level Interface
- Joystick Control
- Vicon-Based Position Control
- VIO-Based Position Control

#### Control Through Low-Level Interface
- Walk These Ways RL Controller
- Nonlinear-MPC Whole Body Control



<!-- Simply install for your python interpreter of interest using pip:

```bash
git clone https://github.com/Rooholla-KhorramBakht/B1Py.git 
cd B1Py
<path/to/python> -m pip install .
```

## Getting Started
The following point to the documentations and Jupyter notebook examples for various use cases and procedures:


### Simulation
- [NVIDIA Isaac Sim](docs/ISAACSIM.md)
- [Pybullet]()
- [MuJCO]() -->

<!-- ### State Estimation
- [Contact Estimation]()
- [Odometry]()
- [Occupancy Map]()
- [Elevation Map]()
- [Hierarchical Scene Graph]() -->

<!-- ### Control
- [Unitree Builtin Controller](notebooks/unitree_highlevel_joystick_control.ipynb)
- [Walk These Ways RL Controller]()
- [Linear QP Controller]()
- [SQP NL-MPC]() -->
