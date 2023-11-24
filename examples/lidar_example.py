import time

from B1Py.lidar_bridges import ROSPointCloudListener


def main():
    lidar = ROSPointCloudListener()

    while lidar.listener.points is None:
        continue

    for i in range(100):
        if lidar.listener.new_frame_flag:
            points = lidar.process_points()
            print(f"{i}, {points.shape}")

        time.sleep(0.01)

    lidar.close()


if __name__ == "__main__":
    main()
