from gpiozero import AngularServo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

servo = AngularServo(16, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
print("Run")

try:
	while True:
		position = int(input("Position: "))
		servo.angle = position
except KeyboardInterrupt:
    print("Program stopped")