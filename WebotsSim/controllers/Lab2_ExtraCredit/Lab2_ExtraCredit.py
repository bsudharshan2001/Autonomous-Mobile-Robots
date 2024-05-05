"""Lab2_Task2 controller."""

#Done by Sudharshan Balaji - U15977125 for Lab 2 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab2_Task2/Lab2_Task2.py

import os
import math
from WebotsSim.libraries.MyRobot import MyRobot
import matplotlib.pyplot as plt

os.chdir("../..")

# Initialize the robot by creating an instance of the MyRobot class
robot = MyRobot()

maze_file = [
    'worlds/mazes/Labs/Lab2/Lab2_Task1.xml',
    'worlds/mazes/Labs/Lab2/Lab2_Task2_1.xml',
    'worlds/mazes/Labs/Lab2/Lab2_Task2_2.xml',
    'worlds/mazes/Labs/Lab2/Lab2_EC_1.xml',
    'worlds/mazes/Labs/Lab2/Lab2_EC_2.xml'

]
robot.load_environment(maze_file[4])
# robot.load_environment(maze_file[2])
robot.move_to_start()

# Maximum velocity for the robot's motors
# Max forward/backward motor speed of robot according to documentation is 26 rad/s
max_velocity = robot.max_motor_velocity
print(max_velocity)

def rotate_robot(rotation_speed, rad_angle):
    """
    This function rotates the robot along the desired angle.
    """
    
    # Calculate the distance each wheel needs to travel for the desired rotation
    rotate_distance = abs(rad_angle) * (robot.axel_length / 2)
    
    # Determine the direction of rotation (clockwise or counter-clockwise) and set wheel velocities accordingly
    if rad_angle < 0:
        left_wheel_speed = -rotation_speed
        right_wheel_speed = rotation_speed
    else:
        left_wheel_speed = rotation_speed
        right_wheel_speed = -rotation_speed
    
    # Record the initial encoder reading for the rotation start point
    initial_distance = encoder_reading()
    
    robot.set_left_motors_velocity(left_wheel_speed)
    robot.set_right_motors_velocity(right_wheel_speed)
    
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        current_distance = encoder_reading()
        
        # Calculate the distance traveled since the rotation started
        distance_traveled = abs(current_distance - initial_distance)
        
        if distance_traveled >= rotate_distance:
            robot.set_left_motors_velocity(0)
            robot.set_right_motors_velocity(0)
            break

def encoder_reading():
    """
    Gets the front left motor encoder reading for distance travelled by it
    """
    front_left_dist = (robot.wheel_radius * robot.get_front_left_motor_encoder_reading())
    return front_left_dist

def adjust_velocity(signal):
    """
    This is the saturation function which adjusts the velocity to ensure it remains within the maximum and minimum limits.
    """
    signal = max(min(signal, max_velocity), -max_velocity)
    if abs(signal) == max_velocity:
        signal /= 2
    
    return signal


def proportional_gain(wall_dist,kp):
    """
    Computes the necessary velocity to keep a safe distance from obstacles ahead.
    """
    # Getting the distance from the front obstacles
    front_dist = robot.get_lidar_range_image()[400]

    # Calculating the error in distance
    distance_error = (front_dist - wall_dist)

    # Applying proportional control to determine the velocity
    v_front = kp * distance_error

    print("***********************************************************************************")
    print(f"Front Obstacle Distance: {front_dist:.2f}, Velocity Adjustment: {v_front:.2f}")
    print(f"Front Distance Error: {distance_error:.2f}")

    return v_front


def wall_following(wall):

    # Distance to maintain from the side walls
    wall_dist = 0.4
    # Proportional Gain chosen
    kp=5

    v_front = proportional_gain(0.3,kp)

    # Fetch sensor data to determine the closest distance to the wall
    left_dist = min(robot.get_lidar_range_image()[200:300])
    right_dist = min(robot.get_lidar_range_image()[500:600])

    print(f"Left Wall Distance: {left_dist:.2f}, Right Wall Distance:: {right_dist:.2f}")

    # Decide control logic based on the wall side

    if wall == "R":
        error = wall_dist - right_dist
        # Control Signal = u(t)
        ut=abs(kp*error)
        if error>0:
            v_left = adjust_velocity(v_front - ut)
            v_right = adjust_velocity(v_front)

        elif error<0:
            v_left = adjust_velocity(v_front)
            v_right = adjust_velocity(v_front - ut)

        else:
            v_left = adjust_velocity(v_front)
            v_right = adjust_velocity(v_front)

    elif wall == "L":
        error = wall_dist - left_dist
        ut=abs(kp*error)
        if error>0:
            v_right = adjust_velocity(v_front - ut)
            v_left = adjust_velocity(v_front)

        elif error<0:
            v_right = adjust_velocity(v_front)
            v_left = adjust_velocity(v_front - ut)

        else:
            v_left = adjust_velocity(v_front)
            v_right = adjust_velocity(v_front)
    else:
        print("Invalid Option")

    return v_left, v_right


if __name__ == "__main__":

    while robot.experiment_supervisor.step(robot.timestep) != -1:

        wall = "R"

        # Getting the left and right wheel speed from the wall following function
        v_left, v_right = wall_following(wall)


        robot.set_left_motors_velocity(v_left)
        robot.set_right_motors_velocity(v_right)

        print(f"Motor Speeds: Left={v_left:.2f}, Right={v_right:.2f}")

        lidar_data = robot.get_lidar_range_image()
        front_dist = lidar_data[400] #400 is the index to the front
        right_dist = lidar_data[600] #600 is the index to the right
        left_dist = lidar_data[200] #200 is the index to the left


        if front_dist < 0.5 and right_dist < 0.5 and left_dist < 0.5:
            print("Rotating 180 degrees")
            rotate_robot(max_velocity, math.pi)  # Rotate 180 degrees

        elif front_dist < 0.5 and left_dist < 0.5 and wall == "R":
            print("Rotating -90 degrees")
            rotate_robot(max_velocity, -math.pi / 2)  # Rotate -90 degrees

        elif front_dist < 0.5 and right_dist < 0.5 and wall == "L":
            print("Rotating 90 degrees")
            rotate_robot(max_velocity, math.pi / 2)  # Rotate 90 degrees
            
