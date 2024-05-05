"""Lab2_Task1 controller."""

#Done by Sudharshan Balaji - U15977125 for Lab 2 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab2_Task1/Lab2_Task1.py

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
robot.load_environment(maze_file[0])
robot.move_to_start()

# Maximum velocity for the robot's motors
# Max forward/backward motor speed of robot according to documentation is 26 rad/s
max_rad_vel = 26
max_velocity = max_rad_vel * robot.wheel_radius 


def proportional_gain(target_distance):
    """
    Computes the necessary velocity to keep a safe distance from obstacles ahead.
    """
    # Gain for the proportional control
    # Used specifically to compute the velocity adjustment necessary to maintain 
    # a safe distance from obstacles directly in front of the robot (kpf).
    # Increasing this translates to quicker robot movement
    kpf = 2

    # Getting the minimum distance from the front obstacles

    lidar_data = robot.get_lidar_range_image()
    front_distance = min(-1*lidar_data[350], -1*lidar_data[400], -1*lidar_data[450])

    # Calculating the error in distance
    distance_error = -1*target_distance - front_distance
    # Applying proportional control to determine the velocity
    v_front = kpf * distance_error

    print(f"Front Obstacle Distance: {front_distance:.2f}, Velocity Adjustment: {v_front:.2f}")
    # return v_front
    return v_front, front_distance

def adjust_velocity(signal):
    """
    This is the saturation function which adjusts the velocity to ensure it remains within the maximum and minimum limits.
    """
    return max(min(signal, max_velocity), -max_velocity)

def control_avoidance():
    """
    Determines motor velocities to avoid walls using a control strategy.
    """
    # Minimum Distance to maintain from side walls
    wall_dist = 0.30
    # This kps (gain) is used to adjust the robot's left and right motor velocities based on the robot's distance to walls on its sides
    kps = 2
    # Velocity reduced due to front obstacles - here our target is 0.5
    v_front, front_distance = proportional_gain(0.5)

    # Detecting distances to the left and right walls
    lidar_data = robot.get_lidar_range_image()
    left_dist = min(lidar_data[100], lidar_data[200], lidar_data[300])
    right_dist= min(lidar_data[500], lidar_data[600], lidar_data[700])

    print(f"Left Wall Distance: {left_dist:.2f}, Right Wall Distance: {right_dist:.2f}")

    # Adjusting velocities based on proximity to walls
    if left_dist < wall_dist:
        error = wall_dist - left_dist
        motor_velocity_left = adjust_velocity(v_front)
        motor_velocity_right = adjust_velocity(v_front - abs(kps * error))

    elif right_dist < wall_dist:
        error = wall_dist - right_dist
        motor_velocity_left = adjust_velocity(v_front - abs(kps * error))
        motor_velocity_right = adjust_velocity(v_front)

    else:
        motor_velocity_left = adjust_velocity(v_front)
        motor_velocity_right = adjust_velocity(v_front)

    return motor_velocity_left, motor_velocity_right, front_distance

if __name__ == "__main__":
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        v_left, v_right, front_distance = control_avoidance()

        # Stop the robot if the front distance is within the target range
        if -0.50 <= front_distance <= -0.45:
            robot.set_left_motors_velocity(0)
            robot.set_right_motors_velocity(0)
            print("Target front distance reached. Stopping robot.")
            break

        # Calculating the angular velocities for wheel motors
        angular_velocity_left = v_left / robot.wheel_radius
        angular_velocity_right = v_right / robot.wheel_radius

        # Updating motor velocities
        robot.set_left_motors_velocity(angular_velocity_left)
        robot.set_right_motors_velocity(angular_velocity_right)

        print(f"Motor Velocities -> Left: {angular_velocity_left:.2f} rad/s, Right: {angular_velocity_right:.2f} rad/s")

        
