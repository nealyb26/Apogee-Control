import numpy as np

class EKF:
    def __init__(self, dt=1/100, mass=1.0, Cd=0.5, area=0.01, rho=1.225):
        self.dt = dt  # Time step (s)

        # Rocket parameters
        self.m = mass  # kg
        self.Cd = Cd   # Drag coefficient
        self.A = area  # Cross-sectional area (m^2)
        self.rho = rho # Air density (kg/m^3)
        self.g = 9.81  # Gravity (m/s^2)

        # Initial state: [altitude, velocity]
        self.x = np.array([[0.0], [0.0]])

        # Initial covariance
        self.P = np.eye(2) * 1.0

        # Process noise (tune these!)
        self.Q = np.array([[0.01, 0],
                           [0, 5.0]])  # Higher velocity process noise allows for coast-phase variability

        # Measurement noise (altimeter variance)
        self.R = np.array([[50**2]])  # Adjust based on your altimeter noise (e.g., 50m std dev)

        # Observation matrix (altitude only)
        self.H = np.array([[1, 0]])

    def predict(self):
        alt = self.x[0, 0]
        vel = self.x[1, 0]

        # Compute drag force per unit mass
        drag = 0.5 * self.Cd * self.rho * self.A * vel**2 / self.m
        drag *= np.sign(vel)  # Ensure drag always opposes motion

        # State prediction (nonlinear)
        alt_new = alt + self.dt * vel
        vel_new = vel + self.dt * (-self.g - drag)

        self.x[0, 0] = alt_new
        self.x[1, 0] = vel_new

        # Jacobian of the dynamics (F matrix)
        d_drag_dv = -np.sign(vel) * (self.Cd * self.rho * self.A / self.m) * abs(vel)
        F = np.array([[1, self.dt],
                      [0, 1 + self.dt * d_drag_dv]])

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_measured):
        """
        :param z_measured: Altitude measurement (float)
        :return: (altitude_estimate, velocity_estimate)
        """
        z = np.array([[z_measured]])

        # Measurement residual
        y = z - (self.H @ self.x)

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update
        self.P = (np.eye(2) - K @ self.H) @ self.P

        return float(self.x[0, 0]), float(self.x[1, 0])
