import math
import time
from IMU import VN100IMU
from Kalman import KalmanFilter
#from EKF import ExtendedKalmanFilter
from collections import deque
import os
import threading
import csv
import signal
import sys
from Servo import SinceCam
from Rk4_v1 import Rk4
from erFilter import DataProcessor
#############################################################################
"""
Deploys fins based on altitude trigger or RK4 apogee prediction, 
Kalman and erFilter
Has launch detection at 3 G for 50 points (0.5s) 
"""
# Conversions
in_to_m = 0.0254
ft_to_m = 0.3048
m_to_ft = 3.28084
lb_to_kg = 0.453592
kg_to_lb = 1/lb_to_kg
ft2_to_m2 = 0.092903
g_to_kg = 0.001
# Constants
LOGGER_BUFFER = 18000  # 3 minutes (100 Hz)
LAUNCH_ACCELERATION = 3 # In g
PROPELLANT_MASS = 247.2 * g_to_kg # kg
ROCKET_DRY_MASS = (13.2 *lb_to_kg) - PROPELLANT_MASS # DRY MASS in kg
ROCKET_DIAMETER = 4.014 * in_to_m # m
ROCKET_AREA = (math.pi/4) * (ROCKET_DIAMETER)**2 # m^2
ACS_CD = 5 # CD of rocket with fins deployed
TRIGGER_ALTITUDE = 400
TARGET_APOGEE = 800
TARGET_FREQ = 100 # main loop frequency
INTERVAL = 1 / TARGET_FREQ
PRINT_INTERVAL = 1 # print every 1s
IMU_INTERVAL = 1/200 # IMU runs at 200 Hz
PREWRITE_INTERVAL = 10  # Limit writes to pre_file every 10 seconds
POSTWRITE_INTERVAL = 10  # Limit writes to post_file every 10 seconds
#############################################################################

def calculate_ground_altitude(imu):
    print("Calculating ground altitude...")
    altitude_readings = []

    for _ in range(50):
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
                   "Pressure", "Altitude",
                   "launchDetected"
                   "kf_velocity", "kf_altitude",
                   "er_velocity", "er_altitude", 
                   "predicted_apogee", "triggerAltitudeAchieved"]
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
            # debug print
            #print(f"[imu_reader] Appended altitude: {data.altitude:.2f}")
        else:
            pass
            print("[imu_reader] No new data")
        time.sleep(IMU_INTERVAL * 0.95)


def data_logging_process(imu_deque, stop_event, groundAltitude, trigger_flag, ekf, servoMotor, launched_flag):
    output_directory = os.path.join("IMU_DATA")
    os.makedirs(output_directory, exist_ok=True)

    pre_file = os.path.join(output_directory, "data_log_pre.csv")
    post_file = os.path.join(output_directory, "data_log_post.csv")
    output_file = os.path.join(output_directory, "data_log_combined.csv")

    pre_trigger_buffer = deque(maxlen=12000)
    post_trigger_buffer = []

    last_time = time.perf_counter()
    last_print_time = 0
    last_prewrite_time = 0
    last_postwrite_time = 0

    consecutive_readings_gs = 0
    consecutive_readings_alt = 0
    consecutive_readings_rk4 = 0
    required_consecutive = 50

    #initialize RK4 variables
    apogee_prediction_m = 0
    apogee_prediction_ft = 0

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
        current_altitude = imu_data.altitude

        #### erFilter ####
        data_processor.process_data(current_time, current_altitude)
        altitude_er, velocity_er = data_processor.get_results()

        #### Kalman Filter ####
        #dt = current_time - last_time
        #ekf.predict(dt)
        altitude_kf, velocity_kf= kf.update(current_altitude)

        # Launch detection logic
        if not launched_flag[0]:
            if abs(imu_data.a_x) > LAUNCH_ACCELERATION:
                consecutive_readings_gs += 1
            else:
                consecutive_readings_gs = 0
            if consecutive_readings_gs >= required_consecutive:
                launched_flag[0] = True
                print(f"[Launch Detected] acceleration = {imu_data.a_x:.2f} which exceeds 3g")
                launch_time = current_time
        
        # Check to ensure no premature fin deployment from RK4
        if launched_flag[0]:
            #### RK4 MODEL #################################################################################
            apogee_prediction_m = Rk4_model.rk4_apogee_predictor(current_altitude * ft_to_m, velocity_kf * ft_to_m)
            apogee_prediction_ft = apogee_prediction_m * m_to_ft # conversion to feet (ft)
            #################################################################################################

        # Print at a reduced rate
        if current_time - last_print_time >= PRINT_INTERVAL:
            print(f"[EKF] dt = {dt:.4f} s")
            print(f"[IMU] Altitude={current_altitude:.2f} ft, Velocity={velocity_kf:.2f} ft/s")
            print(f"[RK4] Projected Apogee = {apogee_prediction_ft:.2f} ft")
            last_print_time = current_time

        #### Trigger logic ####################################################################
        if not trigger_flag[0]:
            if current_altitude > groundAltitude + TRIGGER_ALTITUDE:
                consecutive_readings_alt += 1
            else:
                consecutive_readings_alt = 0

            if apogee_prediction_ft > groundAltitude + TARGET_APOGEE:
                consecutive_readings_rk4 += 1
            else:
                consecutive_readings_rk4 = 0

            # Fin trigger based on either alt check or goal apogee achieved
            if (consecutive_readings_alt >= (required_consecutive / 10)) or (consecutive_readings_rk4 >= (required_consecutive / 10 )): # 5 data points
                trigger_flag[0] = True
                print("[Trigger] Target Altitude Achieved!")
                servoMotor.set_angle(45)
        ########################################################################################
        
        # Format the data row
        data_row = [
            f"{current_time:.6f}",
            f"{imu_data.yaw:.2f}", f"{imu_data.pitch:.2f}", f"{imu_data.roll:.2f}",
            f"{imu_data.a_x:.2f}", f"{imu_data.a_y:.2f}", f"{imu_data.a_z:.2f}",
            f"{imu_data.pressure:.2f}", f"{imu_data.altitude:.2f}",
            f"{int(launched_flag[0])}" 
            f"{velocity_kf:.2f}", f"{altitude_kf:.2f}",
            f"{velocity_er:.2f}", f"{altitude_er:.2f}"
            f"{apogee_prediction_ft:.2f}", f"{int(trigger_flag[0])}"
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

        last_time = current_time

    # Final flush after loop ends
    flush_pre_buffer()
    flush_post_buffer()
    combine_files(pre_file, post_file, output_file)
    print(f"[Shutdown] Data logging completed. Combined logs into {output_file}")




if __name__ == "__main__":
    launched_flag = [False]
    imu = VN100IMU()
    time.sleep(1.0)
    servoMotor = SinceCam()
    servoMotor.set_angle(0)

    kf = KalmanFilter(dt=INTERVAL)
    #ekf = ExtendedKalmanFilter(mass = ROCKET_DRY_MASS, area = ROCKET_AREA, Cd = ACS_CD)
    Rk4_model = Rk4(frequency=10, coeeff_drag=ACS_CD, mass=ROCKET_DRY_MASS, area=ROCKET_AREA) # 10 Hz for simulation loop (dt = 0.1)
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
                                            triggerAltitudeAchieved, ekf, servoMotor, launched_flag))

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
