import numpy as np
from collections import deque

class DataProcessor:
    def __init__(self, evan_length=300, velocity_gap=10):
        self.evan_length = evan_length
        self.velocity_gap = velocity_gap
        self.altitude_data = deque(maxlen=evan_length)
        self.time_data = deque(maxlen=evan_length)
        self.smoothed_alt = float('nan')  # Initialize as NaN indicating no value yet
        self.velocity = float('nan')      # Initialize as NaN indicating no value yet

    def process_data(self, time, alt):
        # Add new data to the deques
        self.altitude_data.append(alt)
        self.time_data.append(time)
        
        # Calculate smoothed altitude if we have enough data
        if len(self.altitude_data) == self.evan_length:
            self.smoothed_alt = np.mean(self.altitude_data)

            if len(self.time_data) >= 2 * self.velocity_gap:
                self.calculate_velocity()

    def calculate_velocity(self):
        # Calculate velocities using the smoothed data
        idx1 = -2 * self.velocity_gap
        idx2 = -self.velocity_gap
        idx3 = -self.velocity_gap + 1
        idx4 = -1
        
        A1, A2 = self.altitude_data[idx1], self.altitude_data[idx2]
        t1, t2 = self.time_data[idx1], self.time_data[idx2]
        A3, A4 = self.altitude_data[idx3], self.altitude_data[idx4]
        t3, t4 = self.time_data[idx3], self.time_data[idx4]

        v1 = (A2 - A1) / (t2 - t1)
        v2 = (A4 - A3) / (t4 - t3)
        
        self.velocity = (v1 + v2) / 2

    def get_results(self):
        return self.smoothed_alt, self.velocity
        