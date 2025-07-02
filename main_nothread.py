import math
import time
import os
from collections import deque
from IMU import VN100IMU
from Kalman import KalmanFilter
from Servo import SinceCam

#############################################################################
# Constants
LOGGER_BUFFER = 18000  # 3 minutes (100 Hz)
launchAcceleration = 3
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
    headers = "Time,Yaw,Pitch,Roll,a_x,a_y,a_z,Temperature,Pressure,Altitude,kf_velocity,kf_altitude,triggerAltitudeAchieved\n"
    with open(output_file, "w") as out_file:
        out_file.write(headers)
        with open(pre_file, "r") as pre_f:
            out_file.write(pre_f.read())
        with open(post_file, "r") as post_f:
            out_file.write(post_f.read())

def main():
    imu = VN100IMU()
    servoMotor = SinceCam()
    kf = KalmanFilter(dt=1/160)  # Sample rate is 160 Hz
    triggerAltitudeAchieved = False

    # Initial setup
    servoMotor.set_angle(0)
    
    # Calculate ground altitude
    groundAltitude = calculate_ground_altitude(imu)

    # Directory setup
    base_directory = "Apogee-Control"
    output_directory = os.path.join(base_directory, "IMU_DATA")
    os.makedirs(output_directory, exist_ok=True)
    
    # Define file paths
    pre_file = os.path.join(output_directory, "data_log_pre.txt")
    post_file = os.path.join(output_directory, "data_log_post.txt")
    output_file = os.path.join(output_directory, "data_log_combined.txt")

    rolling_buffer = deque(maxlen=12000)  # 2-minute buffer at 100 Hz
    target_frequency = 100  # Hz
    interval = 1 / target_frequency   
    last_logging_time = time.perf_counter()
    landing_event_time = None

    consecutive_readings = 0
    required_consecutive = 5  # Number of consecutive readings needed

    with open(post_file, "w") as post_f:
        pass

    try:
        while True:
            current_time = time.perf_counter()
            #print(f'Time: {current_time:.02f}')
            imu.readData()
            if current_time - last_logging_time >= interval:
                imu.readData()
                time.sleep(0.001)
                if imu.currentData:
                    current_altitude = imu.currentData.altitude

                    # Kalman filter update
                    altitude_estimate, velocity_estimate = kf.update(current_altitude)
                    print(f"Current Altitude: {current_altitude:.2f}")

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

                        # Landing detection placeholder logic
                        if False:
                            if landing_event_time is None:
                                landing_event_time = current_time
                                print("Landing detected. Starting post-landing timeout.")

                            if landing_event_time and current_time - landing_event_time >= 120:
                                print("Timeout reached. Stopping logging process.")
                                break

                last_logging_time = current_time

    except KeyboardInterrupt:
        print("Data monitoring interrupted.")

    combine_files(pre_file, post_file, output_file)
    print(f"Data logging completed. Logs combined into {output_file}")

if __name__ == "__main__":
    main()