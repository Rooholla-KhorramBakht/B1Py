import time

from B1Py.interfaces import B1HighLevelReal


def main():
    robot = B1HighLevelReal()

    for i in range(100):
        time.sleep(0.01)
        robot.setCommand(0.02, 0.0, 0.0)

        accel, gyro, quat, rpy = robot.getIMU()
        ang, vel = robot.getJointStates()
        battery_percentage = robot.getBatteryState()

        tic = time.time()
        in_collision = robot.check_calf_collision(ang)
        toc = time.time()

        print(in_collision, toc - tic)

        if i >= 10 and battery_percentage <= 10:
            print("Battery low, exiting")
            break


if __name__ == "__main__":
    main()
