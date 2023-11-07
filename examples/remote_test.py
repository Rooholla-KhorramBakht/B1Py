import time

from B1Py.interfaces import B1HighLevelReal


def main():
    robot = B1HighLevelReal(vx_max=0.015, vy_max=0.015, ωz_max=0.01)

    for i in range(100):
        time.sleep(0.01)

        v_x, v_y, ω = robot.getCommandFromRemote()
        print(v_x, v_y, ω)
        robot.setCommand(v_x, v_y, ω)

        # robot.setCommand(0.02, 0.0, 0.0)

        battery_percentage = robot.getBatteryState()

        if i >= 10 and battery_percentage <= 10:
            print("Battery low, exiting")
            break


if __name__ == "__main__":
    main()
