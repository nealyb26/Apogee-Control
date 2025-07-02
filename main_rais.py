import math
import time
from IMU import VN100IMU
from Kalman import KalmanFilter
from collections import deque
import os
import threading
import queue
import csv
from Servo import SinceCam

#############################################################################
# Constants
LOGGER_BUFFER = 18000  # 3 minutes (100 Hz)
launchAcceleration = 3
TARGET_FREQ = 100
INTERVAL = 1 / TARGET_FREQ
PRINT_INTERVAL = 0.5
#############################################################################

def calculate_ground_altitude(imu):
    print("Calculating ground altitude...")
    altitude_readings = []

    for _ in range(5):
        while True:
            imu.readData()
            if imu.currentData:
                altitude_readings.append(imu.currentData.altitude)
                break
        time.sleep(0.1)

    groundAltitude = sum(altitude_readings) / len(altitude_readings)
    print(f"Ground Altitude: {groundAltitude:.2f} ft")
    return groundAltitude

def combine_files(pre_file, post_file, output_file):
    with open(output_file, "w", newline='') as out_file:
        writer = csv.writer(out_file)
        headers = ["Time", "Yaw", "Pitch", "Roll", "a_x", "a_y", "a_z",
                   "Temperature", "Pressure", "Altitude",
                   "kf_velocity", "kf_altitude", "triggerAltitudeAchieved"]
        writer.writerow(headers)

        for file_path in [pre_file, post_file]:
            with open(file_path, "r") as in_file:
                for line in in_file:
                    writer.writerow(line.strip().split(","))

def imu_reader(imu, imu_queue, stop_event):
    while not stop_event.is_set():
        imu.readData()
        if imu.currentData:
            imu_queue.put((time.perf_counter(), imu.currentData))
        time.sleep(1/160)  # Match IMU output rate

def data_logging_process(imu_queue, stop_event, groundAltitude, trigger_flag, kf, servoMotor):
    base_directory = "Apogee-Control"
    output_directory = os.path.join("IMU_DATA")
    os.makedirs(output_directory, exist_ok=True)

    pre_file = os.path.join(output_directory, "data_log_pre.csv")
    post_file = os.path.join(output_directory, "data_log_post.csv")
    output_file = os.path.join(output_directory, "data_log_combined.csv")

    rolling_buffer = deque(maxlen=12000)
    last_logging_time = time.perf_counter()
    last_print_time = 0

    consecutive_readings = 0
    required_consecutive = 5

    while not stop_event.is_set():
        try:
            current_time, imu_data = imu_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        if current_time - last_logging_time >= INTERVAL:
            current_altitude = imu_data.altitude
            altitude_estimate, velocity_estimate = kf.update(current_altitude)

            if current_time - last_print_time >= PRINT_INTERVAL:
                print(f"[DataLogger] Altitude={current_altitude:.2f} ft")
                last_print_time = current_time

            if not trigger_flag[0]:
                if current_altitude > groundAltitude + 100:
                    consecutive_readings += 1
                else:
                    consecutive_readings = 0

                if consecutive_readings >= required_consecutive:
                    trigger_flag[0] = True
                    print("Initial Altitude Achieved!")
                    servoMotor.set_angle(180)

            data_row = [
                f"{current_time:.2f}",
                f"{imu_data.yaw:.2f}", f"{imu_data.pitch:.2f}", f"{imu_data.roll:.2f}",
                f"{imu_data.a_x:.2f}", f"{imu_data.a_y:.2f}", f"{imu_data.a_z:.2f}",
                f"{imu_data.temperature:.2f}", f"{imu_data.pressure:.2f}",
                f"{imu_data.altitude:.2f}", f"{velocity_estimate:.2f}", f"{altitude_estimate:.2f}",
                f"{int(trigger_flag[0])}"
            ]

            if not trigger_flag[0]:
                rolling_buffer.append(data_row)
                with open(pre_file, "w", newline='') as pre_f:
                    writer = csv.writer(pre_f)
                    writer.writerows(rolling_buffer)
            else:
                with open(post_file, "a", newline='') as post_f:
                    writer = csv.writer(post_f)
                    writer.writerow(data_row)

            last_logging_time = current_time

    combine_files(pre_file, post_file, output_file)
    print(f"Data logging completed. Logs combined into {output_file}")

if __name__ == "__main__":
    imu = VN100IMU()
    servoMotor = SinceCam()
    servoMotor.set_angle(0)

    kf = KalmanFilter(dt=INTERVAL)
    stop_event = threading.Event()
    triggerAltitudeAchieved = [False]  # Use list for mutability across threads

    groundAltitude = calculate_ground_altitude(imu)

    imu_queue = queue.Queue()

    imu_thread = threading.Thread(target=imu_reader, args=(imu, imu_queue, stop_event))
    logging_thread = threading.Thread(target=data_logging_process,
                                      args=(imu_queue, stop_event, groundAltitude,
                                            triggerAltitudeAchieved, kf, servoMotor))

    imu_thread.start()
    logging_thread.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down...")
        stop_event.set()

    imu_thread.join()
    logging_thread.join()
