from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

# hub.light_matrix.show_image('HAPPY')

# hub.light_matrix.show_image('SAD')
m_Left = Motor('A')
m_Right = Motor('B')

# Initialize the Force Sensor
force_sensor = ForceSensor('E')

# Initialize the Color Sensor
color_sensor = ColorSensor('C')


driving_speed = 25
turning = 360

# hub.light_matrix.write(driving_speed)

# m_Left.set_relative_position(0)
# m_Right.set_relative_poistion(0)

print("Assigning m_Drive")


m_Drive = MotorPair('A', 'B')  # Set the motor ports in the motor_pair.
m_Drive.set_default_speed(50)  # Set the default speed of the motor_pair.

# Tank track gear-wheel is 1.5 in / 3.81 cm
# Tank Track Rotation = 3.81 * π = 11.969468 cm
w_base_in = 1.5
w_base_cm = 3.81
# w_rotation = 1.5 * pi;

# print(w_rotation)

while True:
    if force_sensor.is_pressed():
        hub.light_matrix.show_image('SAD')
    else:
        hub.light_matrix.show_image('HAPPY')

    color = color_sensor.wait_for_new_color()
    if color == 'yellow':
        hub.status_light.on('yellow')
    elif color == 'red':
        hub.status_light.on('red')
    elif color == 'blue':
        hub.status_light.on('blue')
    else:
        hub.status_light.on('white')


# m_Drive.set_motor_rotation(11.969468,'cm')
# m_Drive.set_motor_rotation(17.6, 'cm') # Set the distance that the robot travels for one rotation of its wheels.
        # The value 17.6 comes from the diameter of the wheel (5.6cm) multiplied by "π" (3.14).

# m_Drive.move(50,'cm', 0, 50)     # Start moving for 50cm with NO Steering '0' at 50% of the maximum speed.
# m_Drive.move(8.8,'cm', -100, 50) # Start moving for 8.8cm with steering '-100' at 50% of the maximum speed. This makes the robot turn left 90deg
        # at 50% of the maximum speed of the motors. The value 8.8 represents the angle of rotation. This is the distance every
        # wheel needs to travel to rotate the robot 90deg. This value is calculated in the following way: The wheel base
        # (the distance between the points where the wheels touch the ground) is the
        # diameter of the turn (in this case it is 11,2cm). Since we want to turn 90deg (1/4 of the turn) we multiply
        # the diameter (11,2) by "π" (3.14) and divide it to 4 as 90 degrees are 1/4 of the circle.
        # Tank Track:
# m_Drive.move(40,'cm', 0, 50)     # Start moving for 40cm with NO Steering '0' at 50% of the maximum speed.
# m_Drive.move(8.8,'cm', 100, 50)  # Start moving for 8.8cm with steering '100' at -50% of the maximum speed. This makes the robot turn right 90deg
        # at 50% of the maximum speed of the motors. Turn right because the steering is positive 100%.

raise SystemExit  # Close the program.
