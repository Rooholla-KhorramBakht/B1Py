import time

import numpy as np

from B1Py.interfaces import B1HighLevelReal


def main():
    """
    Script for testing the velocity clipping of the robot.
    When setting vx_max=0.015, vy_max=0.015, ωz_max=0.001,
    the robot should walk in place.

    Test the robot by running: robot.setCommand(1.0, 1.0, 1.0)
    """
    robot = B1HighLevelReal(test=False, vx_max=0.015, vy_max=0.015, ωz_max=0.001)

    for i in range(1000):
        time.sleep(0.01)

        robot.setCommand(1.0, 1.0, 1.0)

    print("Done")


if __name__ == "__main__":
    main()
