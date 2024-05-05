
#Done by Sudharshan Balaji - U15977125 for Lab 1 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab1_Task1/Lab1_Task1.py

import os
import math
from WebotsSim.libraries.MyRobot import MyRobot
import matplotlib.pyplot as plt

os.chdir("../..")

# Initialize the robot by creating an instance of the MyRobot class
robot_instance = MyRobot()

maze_path = 'worlds/mazes/Labs/Lab1/Lab1_Task1.xml'
robot_instance.load_environment(maze_path)
robot_instance.move_to_start()

# I initially define a global value of the start of the robot and update it with each movement completion using calculate_new_position()
# Initial coordinates of the robot and previous encoder value for position tracking
timestamps = []  # List to store timestamps
speeds = []  # List to store speeds
starttime = 0
x_coord = 1.5  # Initial X-coordinate
y_coord = -1.5  # Initial Y-coordinate
previous_encoder_val = 0  # Previous encoder reading

# Robot's manual or constant parameters for movement calculations
angular_speed = 3  # Angular speed in radians per second
wheel_speed = angular_speed * robot_instance.wheel_radius  # Linear speed of the robot's wheel
mid_axel_dist = robot_instance.axel_length / 2  # Distance from the mid-point of the robot to its axel




def calculate_new_position():
    """
    Calculates and updates the robot's current position based on encoder readings and compass heading.
    """
    global x_coord, y_coord, previous_encoder_val
    heading = robot_instance.get_compass_reading()  # Get the current compass heading

    encoder_readings = robot_instance.get_encoder_readings()  # Get the current encoder readings
    current_encoder_avg = sum(encoder_readings) / len(encoder_readings)  # Calculate the average encoder value

    # Calculate the distance traveled since the last update
    traveled_dist = (current_encoder_avg - previous_encoder_val) * robot_instance.wheel_radius
    heading_rad = math.radians(heading)

    # Update the robot's coordinates based on the distance traveled and the heading
    x_coord += traveled_dist * math.cos(heading_rad)
    y_coord += traveled_dist * math.sin(heading_rad)

    print(f"--------- x_coord={round(x_coord, 1)}------- y_coord={round(y_coord, 1)}---------θ={heading}--------")

    previous_encoder_val = current_encoder_avg  # Update the previous encoder value for the next calculation

# def navigate_ellipse(a, b):
#     """
#     Navigates the robot along an elliptical path. WIP :)
    
#     --a: Semi-major axis of the ellipse.
#     --b: Semi-minor axis of the ellipse.
#     """
#     total_points = 5  
    
#     for step in range(total_steps):
#         theta = math.radians(step * (180 / total_steps))
        
#         # Calculate curvature (k) at the current point
#         k = ((8*(y_coord)^2/((a^2)*(b^4))) +  (8*(x_coord)^2/((b^2)*(a^4))))/ (math.sqrt(((4*(x_coord)^2)/(a^4)) +  ((4*(y_coord)^2)/(b^4))))^3
        
#         while robot_instance.experiment_supervisor.step(robot_instance.timestep) != -1:
#             front_left_wheel_dist = robot_instance.wheel_radius * robot_instance.get_front_left_motor_encoder_reading()
#             calculate_new_position()
#             if front_left_wheel_dist > dist_traveled:
#                 return dist_traveled

#     robot_instance.stop()

def navigate_curve(dist_traveled, curve_side, curve_point):
    """
    Navigates the robot along a curve based on specified parameters.
    
    --dist_traveled: The distance already traveled by the robot.
    --curve_side: The side of the robot (left or right) that will be inside the curve.
    --curve_point: An identifier for the type of curve to navigate.

    """
    circle_radii = {0: 0.5, 1: 0.5, 2: 1.5, 3: 0.75}
    radius = circle_radii[curve_point] 

    # Determine the fraction of the curve to navigate based on the curve point
    if curve_point in [0, 1]:
        curve_fraction = 1/4
    elif curve_point == 2:
        curve_fraction = 1/4
    elif curve_point == 3:
        curve_fraction = 1/2

    # Calculate distances for inner and outer paths of the curve and the total curve length
    inner_dist = (2 * math.pi * (radius - mid_axel_dist)) * curve_fraction
    outer_dist = (2 * math.pi * (radius + mid_axel_dist)) * curve_fraction
    curve_length = (2 * math.pi * radius) * curve_fraction

    # Calculate curve angular speed and the linear speed of the inner wheel
    curve_angular_speed = wheel_speed / (radius + mid_axel_dist)
    inner_wheel_speed = curve_angular_speed * (radius - mid_axel_dist)
    inner_wheel_angular_velocity = inner_wheel_speed / robot_instance.wheel_radius

    # Set the velocity of the motors based on the curve side (left or right)
    if curve_side == "anticlockwise":
        robot_instance.set_right_motors_velocity(angular_speed)
        robot_instance.set_left_motors_velocity(inner_wheel_angular_velocity)
        dist_traveled += inner_dist
    elif curve_side == "clockwise":
        robot_instance.set_right_motors_velocity(inner_wheel_angular_velocity)
        robot_instance.set_left_motors_velocity(angular_speed)
        dist_traveled += outer_dist
    else:
        print("Invalid curve direction")
        robot_instance.stop()


    print(f"------------Curve: R={radius}m,------Time={round(curve_length / ((inner_wheel_speed + wheel_speed) / 2), 1)}s-------")

    while robot_instance.experiment_supervisor.step(robot_instance.timestep) != -1:
        front_left_wheel_dist = robot_instance.wheel_radius * robot_instance.get_front_left_motor_encoder_reading()
        calculate_new_position()
        if front_left_wheel_dist > dist_traveled:
            return dist_traveled

def navigate_straight_line(dist_traveled, dist_to_cover):
    """
    Navigates the robot in a straight line for a specified distance.
    
    --dist_traveled: The distance already traveled by the robot.
    --dist_to_cover: The additional distance to travel in a straight line.

    """
    global timestamps, speeds
    # Set the velocity of the motors to move straight
    robot_instance.set_right_motors_velocity(angular_speed)
    robot_instance.set_left_motors_velocity(angular_speed)

    dist_traveled += dist_to_cover  # Update the total distance traveled
    time_to_travel = dist_to_cover / wheel_speed  # Calculate the time to travel the additional distance

    print(f"---------Straight Line: Distance={round(dist_to_cover, 1)}m,--------- Time={round(time_to_travel, 1)}s-------")

    while robot_instance.experiment_supervisor.step(robot_instance.timestep) != -1:
        front_left_wheel_dist = robot_instance.wheel_radius * robot_instance.get_front_left_motor_encoder_reading()
        calculate_new_position()
        if front_left_wheel_dist > dist_traveled:
            return dist_traveled

def plot_graph():
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, speeds, label='Speed')
    plt.xlabel('Time (s)')
    plt.ylabel('velocity (m/s)')
    plt.title('Robot velocity Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    total_traveled_dist = 0  # Initialize the total distance traveled

    points = [(2.5, "straight"), (0, "curve"), (2.0, "straight"), (1, "curve"), (1, "straight"), (2, "curve"), (3, "curve")]

    for point in points:
        if point[1] == "straight":
            print(f"Moving from Point to Point: Straight Line")
            total_traveled_dist = navigate_straight_line(total_traveled_dist, point[0])
        elif point[1] == "curve":
            print(f"Curving around Point")
            total_traveled_dist = navigate_curve(total_traveled_dist, "clockwise", point[0])
    
    # plot_graph()

    robot_instance.stop()
