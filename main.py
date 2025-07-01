import math
import time
from IMU import VN100IMU
from Kalman import KalmanFilter
from collections import deque

import os
import threading

from Servo import SinceCam

#############################################################################
# Constants
LOGGER_BUFFER = 18000 # 3 minutes (100 Hz)
launchAcceleration = 3
#############################################################################
"""
def imu_data_process(imu, imu_data):
    '''
    Process to continuously read data from IMU and update imu data
    structure
    '''
    #signal.signal(signal.SIGINT, signal.SIG_IGN)
    #signal.signal(signal.SIGTERM, signal.SIG_IGN)

    while True:

        imu.readData()

        if imu.currentData:
            # updata data
            imu_data[0] = imu.currentData.yaw
            imu_data[1] = imu.currentData.pitch
            imu_data[2] = imu.currentData.roll
            imu_data[3] = imu.currentData.a_x
            imu_data[4] = imu.currentData.a_y
            imu_data[5] = imu.currentData.a_z
            imu_data[6] = imu.currentData.temperature
            imu_data[7] = imu.currentData.pressure
            imu_data[8] = imu.currentData.altitude

def data_logging(imu_data):
    '''
    Logging process with launch detection (based on launchAcceleration) 
    and post-landing timeout.
    Logs pre-launch data (2-minute rolling buffer at 100 Hz) 
    and post-launch data into separate files, then combines them 
    with column headers.
    '''
    
    #signal.signal(signal.SIGINT, signal.SIG_IGN)
    #signal.signal(signal.SIGTERM, signal.SIG_IGN)

    # File names
    pre_file = "data_log_pre.txt"
    post_file = "data_log_post.txt"
    output_file = "data_log_combined.txt"
    # Rolling buffer for pre-launch data (2-minute rolling window at 100 Hz)
    rolling_buffer = deque(maxlen=LOGGER_BUFFER)  # 12000 entries for 2 minutes at 100 Hz
    target_frequency = 100  # Hz
    interval = 1 / target_frequency  # 10 ms
    start_time = time.perf_counter()
    last_logging_time = start_time  # To control logging frequency

    print("Data logging process started.")
# Clear post file at the start to avoid appending old data
    with open(post_file, "w") as post_f:
        pass  # Just open and close to truncate the file

    while True:
        
        current_time = time.perf_counter()  # Current time

        # Ensure consistent logging frequency
        if current_time - last_logging_time >= interval:
            data_str = (
                f"{current_time:.2f},"        # 1  Time
                f"{imu_data[0]:.2f},"         # 2  yaw
                f"{imu_data[1]:.2f},"         # 3  pitch
                f"{imu_data[2]:.2f},"         # 4  roll
                f"{imu_data[3]:.2f},"         # 5  a_x
                f"{imu_data[4]:.2f},"         # 6  a_y
                f"{imu_data[5]:.2f},"         # 7  a_z
                f"{imu_data[6]:.2f},"         # 9  temperature
                f"{imu_data[7]:.2f},"         # 10 pressure
                f"{imu_data[8]:.2f},"         # 11 altitude
                f"{current_velocity.value:.2f}\n"

            )
"""    

# Test script to output the current altitude and vertical velocity
""" if __name__ == "__main__":
    imu = VN100IMU()
    kf = KalmanFilter(dt=1/160)
    try:
        while True:
            if imu.currentData:
                imu.readData()
                alt_estimate, vel_estimate = kf.update(imu.currentData.altitude)
                print(f"Alt:{alt_estimate:.3f} ft, Vel: {vel_estimate:.3f} ft/s")
                
    except KeyboardInterrupt:
        print("Data monitoring interrupted.") """



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
    headers = "Time,Yaw,Pitch,Roll,a_x,a_y,a_z,Temperature,Pressure,Altitude,accel_magnitude,apogee,battery_percentage,survivability_percentage,detection_time_H,detection_time_M,detection_time_S,max_velocity,landing_velocity,current_velocity,landedState,triggerAltitudeAchieved\n"
    with open(output_file, "w") as out_file:
        out_file.write(headers)
        with open(pre_file, "r") as pre_f:
            out_file.write(pre_f.read())
        with open(post_file, "r") as post_f:
            out_file.write(post_f.read())

def data_logging_process(imu, stop_event, groundAltitude, triggerAltitudeAchieved):
    # Define directory
    base_directory = "Apogee-Control"
    output_directory = os.path.join(base_directory, "IMU_DATA")
    os.makedirs(output_directory, exist_ok=True)

    # Define file paths within the directory
    pre_file = os.path.join(output_directory, "data_log_pre.txt")
    post_file = os.path.join(output_directory, "data_log_post.txt")
    output_file = os.path.join(output_directory, "data_log_combined.txt")

    rolling_buffer = deque(maxlen=12000)  # 2-minute buffer at 100 Hz
    target_frequency = 100  # Hz
    interval = 1 / target_frequency  
    start_time = time.perf_counter()
    last_logging_time = start_time
    landing_event_time = None  

    # Counter for consecutive altitude readings
    consecutive_readings = 0
    required_consecutive = 5  # Number of consecutive readings needed

    with open(post_file, "w") as post_f:
        pass

    while not stop_event.is_set():
        current_time = time.perf_counter()

        if current_time - last_logging_time >= interval:
            imu.readData()
            if imu.currentData:
                current_altitude = imu.currentData.altitude

                # Check if altitude condition is met
                if not triggerAltitudeAchieved:
                    if current_altitude > groundAltitude + 100:
                        consecutive_readings += 1
                    else:
                        consecutive_readings = 0

                    if consecutive_readings >= required_consecutive:
                        triggerAltitudeAchieved = True
                        print("Initial Altitude Achieved!")
                        servoMotor.set_angle(180)

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
                    f"{current_altitude:.2f},"
                    "0.00,"  # Placeholder for accel_magnitude
                    "0.00,"  # Placeholder for apogee
                    "0.00,"  # Placeholder for battery_percentage
                    "0.00,"  # Placeholder for survivability_percentage
                    "0,"     # Placeholder for detection_time_H
                    "0,"     # Placeholder for detection_time_M
                    "0,"     # Placeholder for detection_time_S
                    "0.00,"  # Placeholder for max_velocity
                    "0.00,"  # Placeholder for landing_velocity
                    "0.00,"  # Placeholder for current_velocity
                    "0,"     # Placeholder for landedState
                    f"{int(triggerAltitudeAchieved)}\n"  # triggerAltitudeAchieved
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

                    if False:  # Replace this with landing detection logic
                        if landing_event_time is None:
                            landing_event_time = current_time
                            print("Landing detected. Starting post-landing timeout.")

                        if landing_event_time and current_time - landing_event_time >= 120:
                            print("Timeout reached. Stopping logging process.")
                            stop_event.set()
                            break

            last_logging_time = current_time

    combine_files(pre_file, post_file, output_file)
    print(f"Data logging completed. Logs combined into {output_file}")

if __name__ == "__main__":
    imu = VN100IMU()  # Assuming you have this class implemented somewhere
    servoMotor = SinceCam()

    stop_event = threading.Event()
    triggerAltitudeAchieved = False
    servoMotor.set_angle(0)

    # Calculate ground altitude
    groundAltitude = calculate_ground_altitude(imu)

    # Start the data logging in a separate thread
    logging_thread = threading.Thread(
        target=data_logging_process,
        args=(imu, stop_event, groundAltitude, triggerAltitudeAchieved)
    )
    logging_thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(1)  # Main loop operations can be handled here
    except KeyboardInterrupt:
        print("Stopping due to KeyboardInterrupt.")
        stop_event.set()

    logging_thread.join()