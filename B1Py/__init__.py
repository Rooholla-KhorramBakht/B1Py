import os
import platform
import sys

architecture = platform.architecture()[0]
machine = platform.machine()
B1_USD_PATH = os.path.join(os.path.dirname(__file__), "assets/usd/b1.usd")
URDF_DIR_PATH = os.path.join(os.path.dirname(__file__), "assets/urdf")
B1_ISAACSIM_CFG_PATH = os.path.join(os.path.dirname(__file__), "sim/isaac/sim_config.yaml")

if architecture == "64bit":
    if "x86_64" in machine:
        lib_path = os.path.join(os.path.dirname(__file__), "unitree_legged_sdk/amd64")
    elif "aarch64" in machine:
        lib_path = os.path.join(os.path.dirname(__file__), "unitree_legged_sdk/arm64")
    else:
        raise OSError("Unsupported architecture")
    sys.path.append(lib_path)
else:
    raise OSError("Unsupported architecture")
