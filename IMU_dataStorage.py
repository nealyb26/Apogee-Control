import serial
import time
import math
import struct
from dataclasses import dataclass
from collections import deque
import threading

@dataclass
class IMUData:
    yaw: float  # deg
    pitch: float  # deg
    roll: float  # deg
    a_x: float  # G
    a_y: float  # G 
    a_z: float  # G
    temperature: float  # deg C
    pressure: float  # kPa
    altitude: float  # ft. altitude data is derived directly from pressure
    

class VN100IMU:
    """
    VN100IMU class to interface with the VectorNav VN-100 IMU sensor.
    
    This class provides methods to initialize the IMU, read data, validate messages,
    and parse data into structured format. It uses serial communication to receive
    data from the IMU.
    """
    def __init__(self, port='/dev/serial0', baudrate=115200, timeout=1):
        """
        Initializes the VN100IMU class with specified serial port parameters.
        
        :param port: Serial port to connect to the IMU (default: '/dev/serial0').
        :param baudrate: Communication speed (default: 115200).
        :param timeout: Read timeout in seconds (default: 1).
        """
        self.serialConnection = None
        self.currentData = None
        
        self.serialConnection = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        self.initializeIMU()    

    def initializeIMU(self):
        """
        Sends the initialization command to the IMU to configure its output.
        """
        self.serialConnection.write(b"$VNWRG,75,2,5,05,0108,0030*XX\r\n")
        ''' 
        explanation of above command:

        $VNWRG - write register
        75 - the register. register 75 is "binary output message configuration"
        2 - message sent out on port 2 
        5 - frequency. 800 / 5 = 160 Hz
        05 - selecting groups 1 and 3
        0110 - selecting YPR and Acceleration from group 1. convert binary to hex
               (see table 17)
        0030 - selecting temp and pres from group 3. convert binary to hex 
               (see table 17)
        '''

        time.sleep(1)
        print("IMU initialized, starting data monitoring")

    @staticmethod
    def calculateCRC(data, length):
        """
        Calculates the CRC (Cyclic Redundancy Check) for data integrity verification.

        :param data: The data byte array to calculate CRC on.
        :param length: The length of the data.
        :return: Computed CRC value as an integer.
        """
        crc = 0
        for i in range(length):
            crc = (crc >> 8) | (crc << 8) & 0xFFFF
            crc ^= data[i]
            crc ^= (crc & 0xFF) >> 4
            crc ^= (crc << 12) & 0xFFFF
            crc ^= ((crc & 0xFF) << 5) & 0xFFFF
        return crc
    
    def readData(self):
        """
        Reads data from the serial port, validates the message using CRC, 
        and parses it if valid. If data is valid, it stores it in currentData
        and prints it in a single row format.
        """
        if self.serialConnection.in_waiting > 0:
            syncByte = self.serialConnection.read(1)

            if syncByte == b'\xFA':
                message = self.serialConnection.read(39)

                if len(message) == 39:
                    calculatedChecksum = self.calculateCRC(message[:37], 37)
                    receivedChecksum = int.from_bytes(message[37:39], 'big')

                    if calculatedChecksum == receivedChecksum:
                        self.currentData = self.parseMessage(message)
    
    def parseMessage(self, message):
        """
        Parses the raw message bytes into an IMUData structure.
        
        :param message: The raw byte message received from the IMU.
        :return: IMUData object containing parsed IMU values.
        """
        yaw = struct.unpack('<f', message[5:9])[0]
        pitch = struct.unpack('<f', message[9:13])[0]
        roll = struct.unpack('<f', message[13:17])[0]

        a_x = struct.unpack('<f', message[17:21])[0] / 9.80665
        a_y = struct.unpack('<f', message[21:25])[0] / 9.80665
        a_z = struct.unpack('<f', message[25:29])[0] / 9.80665

        temperature = struct.unpack('<f', message[29:33])[0]
        pressure = struct.unpack('<f', message[33:37])[0]

        seaLevelref = 101.325
        altitude = 145366.45 * (1-math.pow(pressure/seaLevelref, 0.190284))

        return IMUData(yaw=yaw,pitch=pitch,roll=roll,
                       a_x=a_x,a_y=a_y,a_z=a_z,
                       temperature=temperature,pressure=pressure,
                       altitude=altitude)
    
    def __del__(self):
        """Destructor to ensure serial connection is closed properly."""
        if self.serialConnection and self.serialConnection.is_open:
            print("Closing serial connection...")
            self.serialConnection.close()

def calculate_ground_altitude(imu, threshold=50):
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
    headers = "Time,Yaw,Pitch,Roll,a_x,a_y,a_z,Temperature,Pressure,Altitude,accel_magnitude,apogee,battery_percentage,survivability_percentage,detection_time_H,detection_time_M,detection_time_S,max_velocity,landing_velocity,current_velocity,landedState,initialAltitudeAchieved\n"
    with open(output_file, "w") as out_file:
        out_file.write(headers)
        with open(pre_file, "r") as pre_f:
            out_file.write(pre_f.read())
        with open(post_file, "r") as post_f:
            out_file.write(post_f.read())

def data_logging_process(imu, stop_event, groundAltitude, initialAltitudeAchieved):
    pre_file = "data_log_pre.txt"
    post_file = "data_log_post.txt"
    output_file = "data_log_combined.txt"

    rolling_buffer = deque(maxlen=12000)  # 2-minute buffer at 100 Hz
    target_frequency = 100  # Hz
    interval = 1 / target_frequency  
    start_time = time.perf_counter()
    last_logging_time = start_time
    landing_event_time = None  

    with open(post_file, "w") as post_f:
        pass

    while not stop_event.is_set():
        current_time = time.perf_counter()

        if current_time - last_logging_time >= interval:
            imu.readData()
            if imu.currentData:
                current_altitude = imu.currentData.altitude

                # Check if altitude condition is met
                if not initialAltitudeAchieved and current_altitude > groundAltitude + 2:
                    initialAltitudeAchieved = True
                    print("Initial Altitude Achieved!")

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
                    f"{int(initialAltitudeAchieved)}\n"  # initialAltitudeAchieved
                )

                if not initialAltitudeAchieved:
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
    imu = VN100IMU()
    
    stop_event = threading.Event()
    initialAltitudeAchieved = False

    # Calculate ground altitude
    groundAltitude = calculate_ground_altitude(imu)

    # Start the data logging in a separate thread
    logging_thread = threading.Thread(
        target=data_logging_process,
        args=(imu, stop_event, groundAltitude, initialAltitudeAchieved)
    )
    logging_thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(1)  # Main loop operations can be handled here
    except KeyboardInterrupt:
        print("Stopping due to KeyboardInterrupt.")
        stop_event.set()

    logging_thread.join()