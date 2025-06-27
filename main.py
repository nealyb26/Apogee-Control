import math
import time
from IMU import VN100IMU
from Kalman import KalmanFilter
from collections import deque

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
if __name__ == "__main__":
    imu = VN100IMU()
    kf = KalmanFilter(dt=1/160)
    try:
        while True:
            if imu.currentData:
                imu.readData()
                alt_estimate, vel_estimate = kf.update(imu.currentData.altitude)
                print(f"Alt:{alt_estimate:.3f} ft, Vel: {vel_estimate:.3f} ft/s")
                
    except KeyboardInterrupt:
        print("Data monitoring interrupted.")