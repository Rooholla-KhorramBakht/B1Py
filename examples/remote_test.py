import time

from B1Py.interfaces import B1HighLevelReal
from B1Py.joysticks import Logitech3DPro


def main():
    robot = B1HighLevelReal(vx_max=0.3, vy_max=0.3, ωz_max=0.2)
    joy = Logitech3DPro(joy_id=0)

    for i in range(10000):
        time.sleep(0.01)

        cmd = joy.readAnalog()

        if i <= 200:
            robot.setCommand(0.0, 0.0, 0.0, mode=2)
        else:
            robot.setCommand(
                -cmd["y"] * robot.vx_max,
                cmd["x"] * robot.vy_max,
                -cmd["z"] * robot.ωz_max,
                mode=2,
            )

        battery_percentage = robot.getBatteryState()

        if i >= 10 and battery_percentage <= 10:
            print("Battery low, exiting")
            break


if __name__ == "__main__":
    main()
