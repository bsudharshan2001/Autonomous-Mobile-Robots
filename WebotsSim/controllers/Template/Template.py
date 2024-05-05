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

# Function to calculate the Euclidean distance between two points
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def print_velocities_and_time(V_left, V_right, distance, time):
    print(f"V_left: {V_left}, V_right: {V_right}, Distance: {distance}, Time: {time}")


def calculate_angle_to_next_waypoint(current_pose, next_waypoint):
    current_x, current_y, current_theta = current_pose
    next_x, next_y, _ = next_waypoint
    
    dx = next_x - current_x
    dy = next_y - current_y
    

    angle_to_waypoint = math.atan2(dy, dx)
    

    relative_angle = angle_to_waypoint - current_theta
    relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
    
    return relative_angle

# Function to move the robot in a straight line
def move_straight(target_distance):

    motor_velocity = MAX_MOTOR_SPEED 
    
    target_encoder_value = target_distance / (math.pi * robot.wheel_radius * 2)
    
    robot.reset_motor_encoders()

    robot.set_left_motors_velocity(motor_velocity)
    robot.set_right_motors_velocity(motor_velocity)

    while True:
        left_encoder = robot.get_front_left_motor_encoder_reading()
        right_encoder = robot.get_front_right_motor_encoder_reading()
        average_encoder_value = (left_encoder + right_encoder) / 2

        if average_encoder_value >= target_encoder_value:
            robot.stop()
            break
      
        robot.experiment_supervisor.step(robot.timestep)

# Function to rotate the robot in place
def rotate_to_target_angle(target_angle):
    initial_bearing = robot.get_compass_reading()


    target_bearing = (initial_bearing + math.degrees(target_angle)) % 360


    clockwise = (target_bearing - initial_bearing + 360) % 360 < 180
    motor_velocity = MAX_MOTOR_SPEED if clockwise else -MAX_MOTOR_SPEED

    robot.set_left_motors_velocity(motor_velocity)
    robot.set_right_motors_velocity(-motor_velocity)

    while True:
        current_bearing = robot.get_compass_reading()

        if clockwise:
            if initial_bearing < target_bearing <= current_bearing:
                break
        else:
            if initial_bearing > target_bearing >= current_bearing:
                break

        if clockwise and current_bearing < initial_bearing:
            initial_bearing -= 360
        elif not clockwise and current_bearing > initial_bearing:
            initial_bearing += 360

        robot.experiment_supervisor.step(robot.timestep)

    robot.stop() 

# Function to follow a circular arc
def follow_arc(angle):

    radius=5

    L = robot.axel_length

    V_outer = MAX_MOTOR_SPEED 
    omega = V_outer / (radius + L/2)

    V_inner = omega * (radius - L/2)


    if angle > 0: 
        robot.set_left_motors_velocity(V_inner)
        robot.set_right_motors_velocity(V_outer)
    else:
        robot.set_left_motors_velocity(V_outer)
        robot.set_right_motors_velocity(V_inner)

    distance_outer = abs(angle) * (radius + L/2)
    distance_inner = abs(angle) * (radius - L/2)

    robot.reset_motor_encoders()

    while True:
        left_encoder = robot.get_front_left_motor_encoder_reading()
        right_encoder = robot.get_front_right_motor_encoder_reading()

        distance_traveled_outer = left_encoder * robot.wheel_radius if angle > 0 else right_encoder * robot.wheel_radius
        distance_traveled_inner = right_encoder * robot.wheel_radius if angle > 0 else left_encoder * robot.wheel_radius

        if distance_traveled_outer >= distance_outer and distance_traveled_inner >= distance_inner:
            robot.stop()
            break

        robot.experiment_supervisor.step(robot.timestep)


for i in range(len(waypoints) - 1):
    current_waypoint = waypoints[i]
    next_waypoint = waypoints[i + 1]


    if(current_waypoint[2]==next_waypoint[2]): 
        move_straight(calculate_distance(current_waypoint,next_waypoint))
    else:
        angle=calculate_angle_to_next_waypoint(current_waypoint,next_waypoint)
        
        rotate_to_target_angle(calculate_angle_to_next_waypoint(current_waypoint,next_waypoint))
        follow_arc(angle)

    # Print velocities and time
    # print_velocities_and_time(V_left, V_right, distance, T_i)
    robot.experiment_supervisor.step(robot.timestep)

robot.stop()





# # Changes Working Directory to be at the root of FAIRIS-Lite
# import os
# os.chdir("../..")

# # Import MyRobot Class
# from WebotsSim.libraries.MyRobot import MyRobot

# # Create the robot instance.
# robot = MyRobot()

# # Loads the environment from the maze file
# maze_file = 'worlds/mazes/Labs/Lab1/Lab1_Task1.xml'
# robot.load_environment(maze_file)

# # Move robot to a random staring position listed in maze file
# robot.move_to_start()

# # Main Control Loop for Robot
# while robot.experiment_supervisor.step(robot.timestep) != -1:

#     print("Max rotational motor velocity: ", robot.max_motor_velocity)

#     # Reads and Prints Distance Sensor Values
#     print("Front Left Distance Sensor: ", robot.get_front_left_distance_reading())
#     print("Front Right Distance Sensor: ", robot.get_front_right_distance_reading())
#     print("Rear Left Distance Sensor: ", robot.get_rear_left_distance_reading())
#     print("Rear Right Distance Sensor: ", robot.get_rear_right_distance_reading())

#     # Reads and Prints Robot's Encoder Readings
#     print("Motor Encoder Readings: ", robot.get_encoder_readings())

#     # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
#     print("Lidar Front Reading", robot.get_lidar_range_image()[400])
#     print("Lidar Right Reading", robot.get_lidar_range_image()[600])
#     print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
#     print("Lidar Left Reading", robot.get_lidar_range_image()[200])
#     print("Simulation Time", robot.experiment_supervisor.getTime())

#     # Sets the robot's motor velocity to 20 rad/sec
#     robot.set_right_motors_velocity(10)
#     robot.set_left_motors_velocity(10)

#     # Calculates distance the wheel has turned since beginning of simulation
#     distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()

#     # Stops the robot after the robot moves a distance of 1.5 meters
#     if distance_front_left_wheel_traveled > 2.5:
#         robot.set_right_motors_velocity(5)
#         robot.set_left_motors_velocity(10)
#         if distance_front_left_wheel_traveled>3.285:
#             robot.set_right_motors_velocity(10)
#             robot.set_left_motors_velocity(10)
#             if distance_front_left_wheel_traveled>5.285:
#                 robot.set_right_motors_velocity(5)
#                 robot.set_left_motors_velocity(10)
#                 if distance_front_left_wheel_traveled>6.05:                     
#                     robot.set_right_motors_velocity(10)
#                     robot.set_left_motors_velocity(10)
#                     if distance_front_left_wheel_traveled>7:
#                         robot.set_right_motors_velocity(5)
#                         robot.set_left_motors_velocity(5.4)
#                         if distance_front_left_wheel_traveled>8.5:
#                             robot.stop()
#                             break
