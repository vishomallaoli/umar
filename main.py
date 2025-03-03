#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Create your objects here.
ev3 = EV3Brick()


# Write your program here.

# Initialize Motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize Sensors
touch_left = TouchSensor(Port.S1)   # Left-side touch sensor
touch_right = TouchSensor(Port.S2)  # Right-side touch sensor
color_sensor = ColorSensor(Port.S3) # Light sensor for tape detection
ultrasonic = UltrasonicSensor(Port.S4) # Ultrasonic sensor for wall/obstacle detection

# Initialize Drive Base
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=120)

# Constants
BASE_SPEED = 150       # Base speed (adjustable)
SAFE_WALL_DIST = 20    # Safe distance from the wall (cm)
OBSTACLE_DIST_THRESH = 30  # Distance at which an object is considered an obstacle (cm)
TURN_ANGLE = 45        # Angle to turn for obstacle avoidance
TURN_SPEED = 100       # Speed for turning
Kp = 1.2               # Proportional gain for line following

# Function to calibrate the light sensor
def calibrate_light_sensor():
    """Determine threshold between blue tape and floor."""
    ev3.speaker.beep()
    floor_reflect = color_sensor.reflection()
    wait(500)
    tape_reflect = color_sensor.reflection()
    return (floor_reflect + tape_reflect) / 2

# Function to determine which side has the wall and which has the tape
def determine_sides():
    """Detect which side has the wall and which side has the tape."""
    drive_base.turn(90)  # Rotate left
    dist_left = ultrasonic.distance()
    
    drive_base.turn(-180)  # Rotate right
    dist_right = ultrasonic.distance()
    
    drive_base.turn(90)  # Return to original forward orientation
    
    if dist_left < dist_right:
        return "left", "right"  # Wall is on the left, tape is on the right
    else:
        return "right", "left"  # Wall is on the right, tape is on the left

# Initial Calibration
threshold = calibrate_light_sensor()
wall_side, tape_side = determine_sides()

# Main Loop for Navigation
while True:
    # Read Sensor Values
    tape_value = color_sensor.reflection()
    dist_ahead = ultrasonic.distance()
    left_bump = touch_left.pressed()
    right_bump = touch_right.pressed()

    # **1. Handle Obstacle Avoidance**
    if dist_ahead < OBSTACLE_DIST_THRESH or left_bump or right_bump:
        drive_base.stop()
        ev3.speaker.beep()

        # If touch sensor triggered, back up slightly
        if left_bump or right_bump:
            drive_base.straight(-50)

        # Choose avoidance direction
        avoid_direction = "right" if wall_side == "left" else "left"
        turn_deg = TURN_ANGLE if avoid_direction == "left" else -TURN_ANGLE

        # Turn and move around obstacle
        drive_base.turn(turn_deg)
        drive_base.straight(100)
        drive_base.turn(-turn_deg)
        continue  # Resume main loop

    # **2. Line Following + Wall Distance Management**
    # Determine line following error
    if tape_side == "left":
        error = tape_value - threshold
    else:
        error = -(tape_value - threshold)

    turn_rate = Kp * error  # Proportional control

    # Wall Distance Adjustment
    if wall_side == "left" and dist_ahead < SAFE_WALL_DIST:
        turn_rate += 20  # Steer right
    elif wall_side == "right" and dist_ahead < SAFE_WALL_DIST:
        turn_rate -= 20  # Steer left

    # Drive robot with speed and correction
    drive_base.drive(BASE_SPEED, turn_rate)

    # **3. Check if Reached End of Track**
    if dist_ahead < 10:  # Assuming wall at the end of track
        ev3.speaker.beep()
        drive_base.stop()
        wait(500)
        drive_base.straight(-100)  # Reverse a bit before turning
        drive_base.turn(180)  # Turn around
        continue  # Resume loop but in the opposite direction
