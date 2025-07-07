import serial
import time
import math
import struct
from dataclasses import dataclass

@dataclass
class IMUData:
    """Data structure to hold IMU readings. (time since startup, 
       RPY, acceleration, temperature, pressure, and altitude) """
    yaw: float # deg
    pitch: float # deg
    roll: float # deg
    a_x: float # G
    a_y: float # G 
    a_z: float # G
    temperature: float # deg C
    pressure: float # kPa
    altitude: float # ft. altitude data is derived directly from pressure
    

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
        
        # Initialize serial connection
        self.serialConnection = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Allow time for the serial connection to initialize
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
        if self.serialConnection.in_waiting > 0:  # Check if there is data in the buffer
            syncByte = self.serialConnection.read(1)  # Read the first byte

            if syncByte == b'\xFA':
                message = self.serialConnection.read(39)  # Read the next 39 bytes

                if len(message) == 39:
                    calculatedChecksum = self.calculateCRC(message[:37], 37)
                    receivedChecksum = int.from_bytes(message[37:39], 'big')

                    if calculatedChecksum == receivedChecksum:
                        self.currentData = self.parseMessage(message)

                        #print(f"IMU Read: Altitude={self.currentData.altitude:.2f} ft")

    def parseMessage(self, message):
        """
        Parses the raw message bytes into an IMUData structure.
        
        :param message: The raw byte message received from the IMU.
        :return: IMUData object containing parsed IMU values.
        """
        # Extract YPR (deg)
        yaw = struct.unpack('<f', message[5:9])[0]
        pitch = struct.unpack('<f', message[9:13])[0]
        roll = struct.unpack('<f', message[13:17])[0]

        # Extract acceleration data (G)
        a_x = struct.unpack('<f', message[17:21])[0] / 9.80665
        a_y = struct.unpack('<f', message[21:25])[0] / 9.80665
        a_z = struct.unpack('<f', message[25:29])[0] / 9.80665

        # Extract pressure data (used for altitude)
        temperature = struct.unpack('<f', message[29:33])[0] #Celcius
        pressure = struct.unpack('<f', message[33:37])[0] # kPa

        # Calculate altitude from pressure (ft MSL). 101.325 kPa ref 
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

# Usage example
if __name__ == "__main__":
    imu = VN100IMU()
    try:
        while True:
            imu.readData()
            # Access currentData directly from other parts of the program if needed
            if imu.currentData:
                print(f"{imu.currentData.yaw:.3f}, {imu.currentData.pitch:.3f}, "
                      f"{imu.currentData.roll:.3f}, "
                      f"{imu.currentData.a_x:.3f}, {imu.currentData.a_y:.3f}, "
                      f"{imu.currentData.a_z:.3f}, {imu.currentData.temperature:.3f}, "
                      f"{imu.currentData.pressure:.3f},"
                      f"{imu.currentData.altitude:.3f}")
                
    except KeyboardInterrupt:
        print("Data monitoring interrupted.")