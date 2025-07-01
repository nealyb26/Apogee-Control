from gpiozero import AngularServo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory


class SinceCam:
    Pos_fly = 0
    Pos_brake = 45

class SinceCam:
    Pos_fly = 0
    Pos_brake = 45

    def __init__(self):
        """
        Initializes a servo connected to the specified pin.
        """
        factory = PiGPIOFactory()
        self.servo = AngularServo(16, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

    def set_angle(self, angle):
        """
        Sets the servo to a specific angle (0 to 180 degrees).
        """
        if 0 <= angle <= 180:
            self.servo.angle = angle
            print(f"Servo set to {angle} degrees")
        else:
            print("Invalid angle. Angle must be between 0 and 180.")

    def relax(self):
        """
        Relaxes the servo, allowing it to move freely.
        """
        self.servo.detach()
        print("Servo detached (relaxed)")

""" Usage:
if __name__ == "__main__":
    # Create an instance of MyServo, assuming servo is on GPIO pin 17
    my_servo_instance = MyServo(17) 

    # Set the servo to 90 degrees
    my_servo_instance.set_angle(90)
    sleep(1)

    # Set the servo to 0 degrees
    my_servo_instance.set_angle(0)
    sleep(1)

    # Relax the servo
    my_servo_instance.relax() """