import numpy as np

# Initial state: altitude=0, velocity=0
x = np.array([[0.0],
              [0.0]])

P = np.eye(2)  # initial covariance

dt = 0.1  # time step in seconds
A = np.array([[1, dt],
              [0,  1]])

H = np.array([[1, 0]])  # only altitude is observed

Q = np.array([[1e-3, 0],
              [0, 1e-2]])

R = np.array([[2.0]])  # altimeter noise variance

def kalman_filter(z, x, P):
    # Predict
    x_pred = A @ x
    P_pred = A @ P @ A.T + Q

    # Update
    y = z - (H @ x_pred)  # residual
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    x_new = x_pred + K @ y
    P_new = (np.eye(2) - K @ H) @ P_pred

    return x_new, P_new

# Example loop simulating incoming altimeter data
import time
import random

while True:
    altimeter_reading = 100 + np.sin(time.time()) + np.random.normal(0, 1)  # noisy altitude
    z = np.array([[altimeter_reading]])

    x, P = kalman_filter(z, x, P)

    print(f"Altitude: {x[0,0]:.2f} m, Vertical Velocity: {x[1,0]:.2f} m/s")
    time.sleep(dt)
