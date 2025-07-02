import math
import time
from IMU import VN100IMU
from Kalman import KalmanFilter
from collections import deque
import os
from Servo import SinceCam

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
    headers = "Time,Yaw,Pitch,Roll,a_x,a_y,a_z,Temperature,Pressure,Altitude,kf_velocity,kf_altitude,triggerAltitudeAchieved\n"
    with open(output_file, "w") as out_file:
        out_file.write(headers)
        with open(pre_file, "r") as pre_f:
            out_file.write(pre_f.read())
        with open(post_file, "r") as post_f:
            out_file.write(post_f.read())

def data_logging_initializing():
    base_directory = "Apogee-Control"
    output_directory = os.path.join("IMU_DATA")
    os.makedirs(output_directory, exist_ok=True)

    pre_file = os.path.join(output_directory, "data_log_pre.txt")
    post_file = os.path.join(output_directory, "data_log_post.txt")
    output_file = os.path.join(output_directory, "data_log_combined.txt")

    rolling_buffer = deque(maxlen=12000)
    target_frequency = 100
    interval = 1 / target_frequency  
    last_logging_time = time.perf_counter()

def data_logging_process(imu, groundAltitude, triggerAltitudeAchieved, kf):
      imu.readData()
            if imu.currentData:
                data_str = (
                    f"{current_time:.2f},"
                    f"{imu.currentData.yaw:.2f},"
                    f"{imu.currentData.pitch:.2f},"
                    f"{imu.currentData.roll:.2f},"
                    f"{imu.currentData.a_x:.2f},"
                    f"{imu.currentData.a_y:.2f},"
                    f"{imu.currentData.a_z:.2f},"
                    f"{imu.currentData.temperature:.2f},"
                    f"{imu.currentData.pressure:.2f},"
                    f"{imu.currentData.altitude:.2f},"
                    f"{velocity_estimate:.2f},"
                    f"{altitude_estimate:.2f},"
                    f"{int(triggerAltitudeAchieved)}\n"
                )

                if not triggerAltitudeAchieved:
                    rolling_buffer.append(data_str)
                    with open(pre_file, "w") as pre_f:
                        pre_f.write("".join(rolling_buffer))
                        pre_f.flush()
                else:
                    with open(post_file, "a") as post_f:
                        post_f.write(data_str)
                        post_f.flush()

if __name__ == "__main__":
    imu = VN100IMU()
    servoMotor = SinceCam()

    stop_event = threading.Event()
    triggerAltitudeAchieved = False
    servoMotor.set_angle(0)

    data_logging_initializing()

    # Initialize Kalman filter
    kf = KalmanFilter(dt=interval)

    groundAltitude = calculate_ground_altitude(imu)

    try:
        while not stop_event.is_set():
            current_time = time.perf_counter()
            
            if current_time - last_logging_time >= interval:
                data_logging_process(imu, stop_event, groundAltitude, triggerAltitudeAchieved, kf)
            pass
    except KeyboardInterrupt:
        print("Stopping due to KeyboardInterrupt.")
        
        combine_files(pre_file, post_file, output_file)
        print(f"Data logging completed. Logs combined into {output_file}")

    
