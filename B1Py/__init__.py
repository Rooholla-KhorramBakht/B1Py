import platform
import sys
import os

architecture = platform.architecture()[0]
machine = platform.machine()
USD_PATH = os.path.join(os.path.dirname(__file__), 'assets/usd/unitree_b1.usd')
if architecture == '64bit':
    if 'x86_64' in machine:
        lib_path = os.path.join(os.path.dirname(__file__), 'unitree_legged_sdk/amd64')
    elif 'aarch64' in machine:
        lib_path = os.path.join(os.path.dirname(__file__), 'unitree_legged_sdk/arm64')
    else:
        raise EnvironmentError("Unsupported architecture")
    sys.path.append(lib_path)
else:
    raise EnvironmentError("Unsupported architecture")