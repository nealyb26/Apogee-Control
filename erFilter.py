import numpy as np
from collections import deque

class DataProcessor:
    def __init__(self, evan_length=450, velocity_gap=15):
        self.evan_length = evan_length
        self.velocity_gap = velocity_gap
        self.altitude_data = deque(maxlen=evan_length)
        self.time_data = deque(maxlen=evan_length)
        self.smoothed_alt = []
        self.smoothed_time = []
        self.velocity = []
        self.v_time = []

    def process_data(self, time, alt):
        self.altitude_data.append(alt)
        self.time_data.append(time)
        
        if len(self.altitude_data) == self.evan_length:
            # Compute smoothed values
            self.smoothed_alt.append(np.mean(self.altitude_data))
            self.smoothed_time.append(np.mean(self.time_data))

            if len(self.smoothed_alt) >= 2 * self.velocity_gap:
                self.calculate_velocity()

    def calculate_velocity(self):
        max_i = len(self.smoothed_alt) - 2 * self.velocity_gap
        for i in range(max_i):
            idx1a = i
            idx1b = i + self.velocity_gap
            idx2a = i + 1
            idx2b = i + 1 + self.velocity_gap

            A1 = self.smoothed_alt[idx1a]
            A2 = self.smoothed_alt[idx1b]
            t1 = self.smoothed_time[idx1a]
            t2 = self.smoothed_time[idx1b]

            A3 = self.smoothed_alt[idx2a]
            A4 = self.smoothed_alt[idx2b]
            t3 = self.smoothed_time[idx2a]
            t4 = self.smoothed_time[idx2b]

            v1 = (A2 - A1) / (t2 - t1)
            v2 = (A4 - A3) / (t4 - t3)

            self.velocity.append((v1 + v2) / 2)
            self.v_time.append((t1 + t4) / 2)

    def get_results(self):
        return {'smoothed_altitude': self.smoothed_alt, 'velocity': self.velocity}
