# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import math
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab1/Lab1_Task1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

waypoints = [
    (1.5, -1.5, math.pi),
    (-1, -1.5, math.pi),
    (-1.5, -1, math.pi/2),
    (-1.5, 1, math.pi/2),
    (-1, 1.5, 0),
    (0, 1.5, 0),
    (1.5, 0, 3*math.pi/2),
    (0, 0, math.pi/2)
]

MAX_MOTOR_SPEED = robot.max_motor_velocity

def calculate_straight_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def move_straight(point1,point2):
    target_distance= calculate_straight_distance(point1,point2)
    print(target_distance)
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        robot.set_front_left_motor_velocity(10)
        robot.set_front_right_motor_velocity(10)
        distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
        if distance_front_left_wheel_traveled > target_distance:
            robot.stop()

def find_arc_radius(point1,point2):
    


    return radius

def move_along_arc(point1,point2):
    r=find_arc_radius(point1,point2)
    circle=(point1[2]-point2[2])/2*math.pi
    L = robot.axel_length
    d=abs(circle)*2*pi*r
    #Rotate clockwise or anti-clockwise
    if(circle>0):
        d_out=abs(circle)*2*pi*(r+L/2)
        d_in=abs(circle)*2*pi*(r-L/2)
    else:
        d_out=abs(circle)*2*pi*(r-L/2)
        d_in=abs(circle)*2*pi*(r+L/2)

    




for i in range(len(waypoints) - 1):
    current_waypoint = waypoints[i]
    next_waypoint = waypoints[i + 1]

    if(current_waypoint[2]==next_waypoint[2]): 
        print("Moving Straight")
        move_straight(current_waypoint,next_waypoint)
    else:
        move_along_arc(current_waypoint,next_waypoint)

    # Print velocities and time
    # print_velocities_and_time(V_left, V_right, distance, T_i)
    robot.experiment_supervisor.step(robot.timestep)

robot.stop()