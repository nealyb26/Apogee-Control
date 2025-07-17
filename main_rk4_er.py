import time
from IMU import VN100IMU
from collections import deque
import os
import threading
import csv
import signal
from Servo import SinceCam
from erFilter import DataProcessor

#############################################################################
# Constants
LOGGER_BUFFER = 18000  # 3 minutes (100 Hz)
LAUNCH_ACCELERATION = 3
GOAL_APOGEE = 5000  # set goal apogee in ft
TARGET_FREQ = 100  # main loop frequency
INTERVAL = 1 / TARGET_FREQ
PRINT_INTERVAL = 1  # print every 1s
IMU_INTERVAL = 1 / 200  # IMU runs at 200 Hz
PREWRITE_INTERVAL = 10  # Limit writes to pre_file every 10 seconds
POSTWRITE_INTERVAL = 10  # Limit writes to post_file every 10 seconds
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
                   "Pressure", "Altitude", "velocity", "smoothed_altitude", 
                   "triggerAltitudeAchieved"]
        writer.writerow(headers)

        for file_path in [pre_file, post_file]:
            if os.path.exists(file_path):
                with open(file_path, "r") as in_file:
                    for line in in_file:
                        writer.writerow(line.strip().split(","))

def imu_reader(imu, imu_deque, stop_event):
    while not stop_event.is_set():
        data = imu.readData()
        if data:
            imu_deque.append((time.perf_counter(), data))
        time.sleep(IMU_INTERVAL * 0.95)

def data_logging_process(imu_deque, stop_event, groundAltitude, trigger_flag, data_processor, servoMotor):
    output_directory = "IMU_DATA"
    os.makedirs(output_directory, exist_ok=True)

    pre_file = os.path.join(output_directory, "data_log_pre.csv")
    post_file = os.path.join(output_directory, "data_log_post.csv")
    output_file = os.path.join(output_directory, "data_log_combined.csv")

    pre_trigger_buffer = deque(maxlen=12000)
    post_trigger_buffer = []

    last_print_time = 0
    last_prewrite_time = 0
    last_postwrite_time = 0

    consecutive_readings = 0
    required_consecutive = 5

    def flush_pre_buffer():
        with open(pre_file, "w", newline='') as pre_f:
            writer = csv.writer(pre_f)
            writer.writerows(pre_trigger_buffer)
        print("[Log] Pre-trigger buffer flushed to disk.")

    def flush_post_buffer():
        if post_trigger_buffer:
            with open(post_file, "a", newline='') as post_f:
                writer = csv.writer(post_f)
                writer.writerows(post_trigger_buffer)
            print(f"[Log] Flushed {len(post_trigger_buffer)} post-trigger rows.")
            post_trigger_buffer.clear()

    next_logging_time = time.perf_counter()

    while not stop_event.is_set():
        current_time = time.perf_counter()
        sleep_time = next_logging_time - current_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        next_logging_time += INTERVAL

        if not imu_deque:
            continue

        # Only keep the newest data
        while len(imu_deque) > 1:
            imu_deque.popleft()

        current_time, imu_data = imu_deque.pop()
        
        # Use the data processor to process current altitude
        data_processor.process_data(current_time, imu_data.altitude)
        results = data_processor.get_results()
        
        if results['smoothed_altitude']:
            smoothed_altitude = results['smoothed_altitude'][-1]
            velocity_estimate = results['velocity'][-1] if results['velocity'] else 0.0
        else:
            continue

        # Print at a reduced rate
        if current_time - last_print_time >= PRINT_INTERVAL:
            print(f"[IMU] Altitude={imu_data.altitude:.2f} ft, Smoothed Altitude={smoothed_altitude:.2f} ft, Velocity={velocity_estimate:.2f} ft/s")
            last_print_time = current_time

        # Trigger logic - actuate servo when altitude prediction is above goal 5 times
        if not trigger_flag[0]:
            if smoothed_altitude > groundAltitude + GOAL_APOGEE:
                consecutive_readings += 1
            else:
                consecutive_readings = 0

            if consecutive_readings >= required_consecutive:
                trigger_flag[0] = True
                print("[Trigger] Target Altitude Achieved!")
                servoMotor.set_angle(45)

        # Format the data row
        data_row = [
            f"{current_time:.6f}",
            f"{imu_data.yaw:.2f}", f"{imu_data.pitch:.2f}", f"{imu_data.roll:.2f}",
            f"{imu_data.a_x:.2f}", f"{imu_data.a_y:.2f}", f"{imu_data.a_z:.2f}",
            f"{imu_data.pressure:.2f}",
            f"{imu_data.altitude:.2f}", f"{velocity_estimate:.2f}", f"{smoothed_altitude:.2f}",
            f"{int(trigger_flag[0])}"
        ]

        if not trigger_flag[0]:
            pre_trigger_buffer.append(data_row)
            if current_time - last_prewrite_time >= PREWRITE_INTERVAL:
                flush_pre_buffer()
                last_prewrite_time = current_time
        else:
            post_trigger_buffer.append(data_row)
            if current_time - last_postwrite_time >= POSTWRITE_INTERVAL:
                flush_post_buffer()
                last_postwrite_time = current_time

    # Final flush after loop ends
    flush_pre_buffer()
    flush_post_buffer()
    combine_files(pre_file, post_file, output_file)
    print(f"[Shutdown] Data logging completed. Combined logs into {output_file}")

if __name__ == "__main__":
    imu = VN100IMU()
    time.sleep(1.0)
    servoMotor = SinceCam()
    servoMotor.set_angle(0)

    data_processor = DataProcessor()  # Initialize DataProcessor
    stop_event = threading.Event()
    triggerAltitudeAchieved = [False]  # Use list for mutability across threads

    def handle_shutdown(signum, frame):
        print("Signal received. Shutting down...")
        stop_event.set()

    # Handle SIGINT and SIGTERM
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    groundAltitude = calculate_ground_altitude(imu)

    imu_deque = deque(maxlen=1000)

    imu_thread = threading.Thread(target=imu_reader, args=(imu, imu_deque, stop_event))
    logging_thread = threading.Thread(target=data_logging_process,
                                      args=(imu_deque, stop_event, groundAltitude,
                                            triggerAltitudeAchieved, data_processor, servoMotor))

    imu_thread.start()
    logging_thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt received.")
        stop_event.set()

    imu_thread.join()
    logging_thread.join()
    print("[Main] Shutdown complete.")