"""Lab5_EC controller."""

#Done by Sudharshan Balaji - U15977125 for Lab 5 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab5_Task1/Lab5_Task1.py

# Occupany Grid

import os
import math
import numpy as np
os.chdir("../..")

from WebotsSim.libraries.MyRobot import MyRobot

robot = MyRobot()

# Loads the environment from the maze file
maze_file = [
    'worlds/mazes/Labs/Lab5/Lab5_SmallMaze1.xml',
    'worlds/mazes/Labs/Lab5/Lab5_SmallMaze2.xml',
    'worlds/mazes/Labs/Lab5/Lab5_LargeMaze.xml',
]
robot.load_environment(maze_file[0])
robot.move_to_start()

# Initialize maze_map as a 2D array
maze_map = np.full((12, 12), ' ')

cells = [(row, col) for row in range(4) for col in range(4)]



start_pos = robot.starting_position
start_x = start_pos.x
start_y = start_pos.y
start_theta = start_pos.theta

# Initial coordinates of the robot and previous encoder value for position tracking
x = start_x
y = start_y

previous_encoder_val = 0

unknown_val = 0.5 # Already given in the Doc
empty_val = 0.3 # Already given in the Doc
occupied_val = 0.6 # Already given in the Doc

c = 0.33
# Per cell in the grid we will have 3x3 subcells
log_odds = np.full((4, 4, 3, 3), np.log(unknown_val / (1 - unknown_val)))

def encoder_reading():
    """
    Gets the front left motor encoder reading for distance travelled by it
    """
    front_left_dist = (robot.wheel_radius * robot.get_front_left_motor_encoder_reading())
    return front_left_dist


def rotate_angle(angle, speed):
    rotate_distance = angle * (robot.axel_length / 2)
    rotate_distance = abs(rotate_distance)
    if angle < 0:
        prev_distance = encoder_reading()
        while robot.experiment_supervisor.step(robot.timestep) != -1:
            cur_distance = encoder_reading()
            if cur_distance > prev_distance - rotate_distance:
                robot.set_left_motors_velocity(-speed)
                robot.set_right_motors_velocity(speed)
            else:
                return
    else:
            prev_distance = encoder_reading()
            while robot.experiment_supervisor.step(robot.timestep) != -1:
                cur_distance = encoder_reading()
                if cur_distance < prev_distance + rotate_distance:
                    robot.set_left_motors_velocity(speed)
                    robot.set_right_motors_velocity(-speed)
                else:
                    return

def rotate_robot(rotation_speed=5):
    robot.set_left_motors_velocity(-rotation_speed)
    robot.set_right_motors_velocity(rotation_speed)

def move_forward(dist, speed):
    prev_encoder_reading = encoder_reading()
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        current_encoder_reading = encoder_reading()
        if current_encoder_reading < prev_encoder_reading + dist:
            robot.set_left_motors_velocity(speed)
            robot.set_right_motors_velocity(speed)
        else:
            return

def adjust_robot_orientation():
    print("***** Adjusting Orientation ******")

    # Define target compass ranges for direction adjustment
    direction_targets = [(85, 95), (175, 185), (265, 275), (355, 365)]

    # Find the current compass reading and determine the target range
    current_compass_reading = robot.get_compass_reading()
    target_range = None
    if 0 <= current_compass_reading < 90:
        target_range = direction_targets[0]
    elif 90 <= current_compass_reading < 180:
        target_range = direction_targets[1]
    elif 180 <= current_compass_reading < 270:
        target_range = direction_targets[2]
    elif 270 <= current_compass_reading < 360:
        target_range = direction_targets[3]

    # Adjust the robot's orientation to the target range
    if target_range:
        while robot.experiment_supervisor.step(robot.timestep) != -1:
            if not target_range[0] <= robot.get_compass_reading() < target_range[1]:
                rotate_robot()
            else:
                return 

def calculate_new_position():
    """
    Calculates and updates the robot's current position based on encoder readings and compass heading.
    """
    global x, y, previous_encoder_val
    heading = robot.get_compass_reading()  # Get the current compass heading

    encoder_readings = robot.get_encoder_readings()  # Get the current encoder readings
    current_encoder_avg = sum(encoder_readings) / len(encoder_readings)  # Calculate the average encoder value

    # Calculate the distance traveled since the last update
    traveled_dist = (current_encoder_avg - previous_encoder_val) * robot.wheel_radius
    heading_rad = math.radians(heading)

    # Update the robot's coordinates based on the distance traveled and the heading
    x += traveled_dist * math.cos(heading_rad)
    y += traveled_dist * math.sin(heading_rad)

    print(f"X = {round(x, 1)}  --  Y = {round(y, 1)}  --  θ ={heading}")

    previous_encoder_val = current_encoder_avg  # Update the previous encoder value for the next calculation


def index_calc(x, y):

    row, column = None, None
    # Map y values to row indices
    if y >= 1:
        row = 0
    elif y >= 0:
        row = 1
    elif y >= -1:
        row = 2
    elif y >= -2:
        row = 3

    # Map x values to column indices    
    if x >= -2 and x < -1:
        column = 0
    elif x < 0:
        column = 1
    elif x < 1:
        column = 2
    elif x < 2:
        column = 3


    current_idx = cells.index((row, column))

    return current_idx

def stop_mapping():
    # Count the number of occurrences of the specific value
    count = np.count_nonzero(log_odds == np.log(unknown_val / (1 - unknown_val)))
    # Calculate the percentage
    percentage = (count / log_odds.size) * 100
    print(f"Percentage left to cover is {round(percentage, 2)}%")
    # if unknown cells are less than or equal to 10% return true
    if percentage <= 10:
        return 1


def map_walls(cell_index):
    # Define the symbols for the map representation
    north, south, west, east, wall = '\u2191', '\u2193', '\u2190', '\u2192', 'W'
    
    # Prior calculation for wall detection
    prior = np.log(unknown_val / (1 - unknown_val))

    # Iterate through the 4D log_odds array
    for x in range(log_odds.shape[0]):
        for y in range(log_odds.shape[1]):
            for sub_x in range(log_odds.shape[2]):
                for sub_y in range(log_odds.shape[3]):
                    log_value = log_odds[x, y, sub_x, sub_y]
                    wall_value = round(prior + calculate_log_odds(occupied_val, unknown_val), 2)
                    
                    # Calculate the 2D index from the 4D index
                    map_x, map_y = x * 3 + sub_x, y * 3 + sub_y

                    if log_value == wall_value:
                        maze_map[map_x, map_y] = wall

                    if (x, y) == cells[cell_index] and sub_x == 1 and sub_y == 1:
                        compass_reading = robot.get_compass_reading()
                        if 85 <= compass_reading < 95:
                            maze_map[map_x, map_y] = north
                        elif 175 <= compass_reading < 185:
                            maze_map[map_x, map_y] = west
                        elif 265 <= compass_reading < 275:
                            maze_map[map_x, map_y] = south
                        else:
                            maze_map[map_x, map_y] = east

    # Print the maze map with borders
    print("+--" + "--" * (len(maze_map[0])//2) + "+")
    for row in maze_map:
        print('|' + ''.join(row) + '|')
    print("+--" + "--" * (len(maze_map[0])//2) + "+")


def calculate_log_odds(dist, cell_dim):
    # Check whether the measured distance suggests a wall or open space in the cell
    if dist < cell_dim:
        # Detected a wall: calculate log-odds for occupied space
        occupied_log_odds = np.log(occupied_val / (1 - occupied_val))
        return occupied_log_odds
    else:
        # Detected no wall: calculate log-odds for free space
        free_space_log_odds = np.log(empty_val / (1 - empty_val))
        return free_space_log_odds


def get_log_odds(sensor_values, current_index):
    # Calculate the prior log-odds based on the unknown probability
    prior_log_odds = np.log(unknown_val / (1 - unknown_val))

    # Determine the specific cell from the grid using the current index
    grid_cell = cells[current_index]
    x_coord, y_coord = grid_cell
    subgrid_log_odds = log_odds[x_coord, y_coord]

    # Update central subcell as empty
    subgrid_log_odds[1, 1] = round(subgrid_log_odds[1, 1] - prior_log_odds + calculate_log_odds(empty_val, c), 2)

    # Process each direction: North, East, South, West
    directions = [(0, 1), (1, 2), (2, 1), (1, 0)]  # (sub_x, sub_y) indices for N, E, S, W
    for idx, (dx, dy) in enumerate(directions):
        # Update log odds for the current direction
        subgrid_log_odds[dx, dy] = round(subgrid_log_odds[dx, dy] - prior_log_odds + calculate_log_odds(sensor_values[idx], c), 2)

        # If the current sensor reading indicates an occupied space, spread this value to adjacent subcells
        if sensor_values[idx] == occupied_val:
            if dx == 0 or dx == 2:  # North or South direction
                subgrid_log_odds[dx, 0], subgrid_log_odds[dx, 2] = subgrid_log_odds[dx, dy], subgrid_log_odds[dx, dy]
            elif dy == 0 or dy == 2:  # West or East direction
                subgrid_log_odds[0, dy], subgrid_log_odds[2, dy] = subgrid_log_odds[dx, dy], subgrid_log_odds[dx, dy]




def get_sensor_reading(index):
    # Gather sensor readings from different directions
    lidar_readings = robot.lidar.getRangeImage()
    zf = occupied_val if lidar_readings[400] > 0.7 else empty_val
    zr = occupied_val if lidar_readings[600] > 0.7 else empty_val
    zb = occupied_val if lidar_readings[0] > 0.7 else empty_val
    zl = occupied_val if lidar_readings[200] > 0.7 else empty_val

    # Determine robot orientation and assign sensor readings accordingly
    compass_direction = robot.get_compass_reading()
    if 85 <= compass_direction < 95:
        sensor_data = [zf, zr, zb, zl]
    elif 175 <= compass_direction < 185:
        sensor_data = [zr, zb, zl, zf]
    elif 265 <= compass_direction < 275:
        sensor_data = [zb, zl, zf, zr]
    else:
        sensor_data = [zl, zf, zr, zb]

    # Update log odds based on the current sensor data
    get_log_odds(sensor_data, index)


def update_cells(index):
    compass_reading = robot.get_compass_reading()
    if 85 <= compass_reading < 95:
        next_cell = index - 4  # North
    elif 175 <= compass_reading < 185:
        next_cell = index - 1  # West
    elif 265 <= compass_reading < 275:
        next_cell = index + 4  # South
    else:
        next_cell = index + 1  # East

    # Ensure next_cell is valid, e.g., within bounds
    index = next_cell
    return next_cell



if __name__ == "__main__":

    cur_pos = index_calc(start_x, start_y)
    print("X:", start_x, "Y:", start_y)
    while robot.experiment_supervisor.step(robot.timestep) != -1:

        get_sensor_reading(cur_pos)

        cell_n = cells[cur_pos]
        x, y = cell_n
        log_odds_snapshot = log_odds[x, y]
        
        print(f"Current Position: Cell Index = {cur_pos}, Coordinates = ({x}, {y})")
        print(f"Compass Heading: {robot.get_compass_reading()} degrees")
        print(f"Log-Odds Values at Current Cell: {log_odds_snapshot}")

        map_walls(cur_pos)

        # Conditionally moving the robot based on sensor data
        lidar_range_image = robot.get_lidar_range_image()
        if lidar_range_image[400] > 1.0:
            move_forward(1.0, 10.0)
        elif lidar_range_image[200] > 1.0:
            rotate_angle(math.pi + math.pi/1.5, 3)
            adjust_robot_orientation()
            move_forward(1.0, 10.0)
        elif lidar_range_image[600] > 1.0:
            rotate_angle(math.pi/1.5, 3)
            adjust_robot_orientation()
            move_forward(1.0, 10.0)
        else:
            rotate_angle(math.pi + math.pi/2, 3)
            adjust_robot_orientation()
            move_forward(1.0, 10.0)

        cur_pos = update_cells(cur_pos)
        calculate_new_position()
        if stop_mapping():
            print("Mapping complete: Less than 10% of the grid remains unknown.")
            break
    robot.stop()


