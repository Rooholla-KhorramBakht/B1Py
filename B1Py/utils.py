import sys
def addROSPath(installation_path):
    """
    Add the ROS installation path to the python path.

    Parameters:
        installation_path (str): Path to the ROS installation directory.
    """
    sys.path.append(installation_path + '/lib/python3/dist-packages')