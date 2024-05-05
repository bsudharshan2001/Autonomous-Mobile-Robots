"""Lab4_Task2 controller."""

#Done by Sudharshan Balaji - U15977125 for Lab 4 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab4_Task2/Lab4_Task2.py

# Wall Localization

import os
import math
os.chdir("../..")

from WebotsSim.libraries.MyRobot import MyRobot

robot = MyRobot()

# Loads the environment from the maze file
maze_file = [
    'worlds/mazes/Labs/Lab4/Lab4_Task1_1.xml',
    'worlds/mazes/Labs/Lab4/Lab4_Task1_2.xml',
    'worlds/mazes/Labs/Lab4/Lab4_Task1_3.xml',
    'worlds/mazes/Labs/Lab4/Lab4_Task2_1.xml',
    'worlds/mazes/Labs/Lab4/Lab4_Task2_2.xml'
]
robot.load_environment(maze_file[3])
robot.move_to_start()

# Get the starting position of the robot
# From the example given in the Lab 4 Document
start_pos = robot.starting_position
start_x = start_pos.x
start_y = start_pos.y
start_theta = start_pos.theta

# Initial coordinates of the robot and previous encoder value for position tracking
x = start_x
y = start_y

previous_encoder_val = 0

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


# This code provides a very simple implementation of the maze and how the maze's walls can be printed visually.
# You may use this as a starting point or develop your own maze implementation.

class Cell:
	def __init__(self, west, north, east, south, visited = False):
		# There are 4 walls per cell
		# Wall values can be 'W', 'O', or '?' (wall, open, or unknown)
		self.west = west
		self.north = north
		self.east = east
		self.south = south
		
		# Store whether or not the cell has been visited before
		self.visited = visited


# Helper function that verifies all the walls of the maze
def detectMazeInconsistencies(maze):
    # Check horizontal walls
    for i in range(3):
        for j in range(4):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 4
            hWall1 = maze[pos1].south
            hWall2 = maze[pos2].north
            assert hWall1 == hWall2, " Cell " + str(pos1) + "'s south wall doesn't equal cell " + str(
                pos2) + "'s north wall! ('" + str(hWall1) + "' != '" + str(hWall2) + "')"

    # Check vertical walls
    for i in range(4):
        for j in range(3):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 1
            vWall1 = maze[pos1].east
            vWall2 = maze[pos2].west
            assert vWall1 == vWall2, " Cell " + str(pos1) + "'s east wall doesn't equal cell " + str(
                pos2) + "'s west wall! ('" + str(vWall1) + "' != '" + str(vWall2) + "')"


# You don't have to understand how this function works
def printMaze(maze, hRes = 4, vRes = 2):
    assert hRes > 0, "Invalid horizontal resolution"
    assert vRes > 0, "Invalid vertical resolution"

    # Get the dimensions of the maze drawing
    hChars = 4 * (hRes + 1) + 2
    vChars = 4 * (vRes + 1) + 1

    # Store drawing into a list
    output = [" "] * (hChars * vChars - 1)

    # Draw top border
    for i in range(1, hChars - 2):
        output[i] = "_"

    # Draw bottom border
    for i in range(hChars * (vChars - 1) + 1, hChars * (vChars - 1) + hChars - 2):
        output[i] = "¯"

    # Draw left border
    for i in range(hChars, hChars * (vChars - 1), hChars):
        output[i] = "|"

    # Draw right border
    for i in range(2 * hChars - 2, hChars * (vChars - 1), hChars):
        output[i] = "|"

    # Draw newline characters
    for i in range(hChars - 1, hChars * vChars - 1, hChars):
        output[i] = "\n"

    # Draw dots inside maze
    for i in range((vRes + 1) * hChars, hChars * (vChars - 1), (vRes + 1) * hChars):
        for j in range(hRes + 1, hChars - 2, hRes + 1):
            output[i + j] = "·"

    # Draw question marks if cell is unvisited
    for i in range(4):
        for j in range(4):
            cellNum = i * 4 + j
            if maze[cellNum].visited:
                continue
            origin = (i * hChars * (vRes + 1) + hChars + 1) + (j * (hRes + 1))
            for k in range(vRes):
                for l in range(hRes):
                    output[origin + k * hChars + l] = "?"

    # Draw horizontal walls
    for i in range(3):
        for j in range(4):
            cellNum = i * 4 + j
            origin = ((i + 1) * hChars * (vRes + 1) + 1) + (j * (hRes + 1))
            hWall = maze[cellNum].south
            for k in range(hRes):
                output[origin + k] = "-" if hWall == 'W' else " " if hWall == 'O' else "?"

    # Draw vertical walls
    for i in range(4):
        for j in range(3):
            cellNum = i * 4 + j
            origin = hChars + (hRes + 1) * (j + 1) + i * hChars * (vRes + 1)
            vWall = maze[cellNum].east
            for k in range(vRes):
                output[origin + k * hChars] = "|" if vWall == 'W' else " " if vWall == 'O' else "?"

    # Print drawing
    print(''.join(output))

# Initialize the maze with a set of walls and visited cells
# The bottom right cell is marked as unvisited and with unknown walls

env = 1

if env == 1:
    maze = [
        Cell('W', 'W', 'O', 'O', False), Cell('O', 'W', 'O', 'W', False), Cell('O', 'W', 'O', 'W', False), Cell('O', 'W', 'W', 'O', False),
        Cell('W', 'O', 'W', 'O', False), Cell('W', 'W', 'O', 'O', False), Cell('O', 'W', 'W', 'O', False), Cell('W', 'O', 'W', 'O', False),
        Cell('W', 'O', 'W', 'O', False), Cell('W', 'O', 'O', 'W', False), Cell('O', 'O', 'O', 'W', False), Cell('O', 'O', 'W', 'W', False),
        Cell('W', 'O', 'O', 'W', False), Cell('O', 'W', 'O', 'W', False), Cell('O', 'W', 'O', 'W', False), Cell('O', 'W', 'W', 'W', False)
    ]
elif env == 2:
    maze = [
        Cell('W', 'W', 'O', 'W', False), Cell('O', 'W', 'O', 'W', False), Cell('O', 'W', 'O', 'O', False), Cell('O', 'W', 'W', 'O', False),
        Cell('W', 'W', 'O', 'O', False), Cell('O', 'W', 'W', 'O', False), Cell('W', 'O', 'W', 'O', False), Cell('W', 'O', 'W', 'O', False),
        Cell('W', 'O', 'W', 'O', False), Cell('W', 'O', 'O', 'W', False), Cell('O', 'O', 'W', 'W', False), Cell('W', 'O', 'W', 'O', False),
        Cell('W', 'O', 'O', 'W', False), Cell('O', 'W', 'O', 'W', False), Cell('O', 'W', 'O', 'W', False), Cell('O', 'O', 'W', 'W', False)
    ]

    
# How to modify a cell
# maze[0].east = 'W'
# maze[0].visited = False



cells = [(row, col) for row in range(4) for col in range(4)]


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

    # if row is None or column is None:
    #     # print(f"Warning: Unable to determine cell for x={x}, y={y}. Using fallback values.")
    #     row = row if row is not None else 0  
    #     column = column if column is not None else 0

    index=cells.index((row, column))

    return index


def calculate_probability(S, Z):
    # Map conditions to probabilities
    # p (z=0 | s=0) = 0.7
    # p (z=1 | s=0) = 0.3
    # p (z=0 | s=1) = 0.1
    # p (z=1 | s=1) = 0.9
    probabilities = {(0, 0): 0.7, (0, 1): 0.3, (1, 0): 0.1, (1, 1): 0.9}

    return probabilities.get((S, Z), 0)


# Helper function to get the wall configuration for a given cell
def get_cell_probabilities(cell):
    west = 1 if maze[cell].west == "W" else 0
    north = 1 if maze[cell].north == "W" else 0
    east = 1 if maze[cell].east == "W" else 0
    south = 1 if maze[cell].south == "W" else 0
    return west, north, east, south

# Calculate probabilities for next and current cells
def get_probability(index):
    # Retrieve lidar sensor readings for obstacle detection
    zl = 0 if robot.lidar.getRangeImage()[200] > 0.7 else 1
    zf = 0 if robot.lidar.getRangeImage()[400] > 0.7 else 1
    zr = 0 if robot.lidar.getRangeImage()[600] > 0.7 else 1

    # Set current cell as visited
    maze[index].visited = True

    # Determine the next cell based on compass reading
    compass_reading = robot.get_compass_reading()
    if compass_reading in range(85, 95):
        next_cell = index - 4  # North
    elif compass_reading in range(175, 185):
        next_cell = index - 1  # West
    elif compass_reading in range(265, 275):
        next_cell = index + 4  # South
    else:
        next_cell = index + 1  # East

    print(f"Current cell: {index}")

    # Calculate probability for current cell based on walls and lidar data
    cur_west = 1 if maze[index].west == "W" else 0
    cur_north = 1 if maze[index].north == "W" else 0
    cur_east = 1 if maze[index].east == "W" else 0

    current_cell_prob = calculate_probability(cur_west, zl) * calculate_probability(cur_north, zf) * calculate_probability(cur_east, zr)

    # Initialize probability list for all cells
    cell_probabilities = [0] * 16
    for i in range(16):
        if i == index:
            continue
        
        other_cell_west = 1 if maze[i].west == "W" else 0
        other_cell_north = 1 if maze[i].north == "W" else 0
        other_cell_east = 1 if maze[i].east == "W" else 0

        cell_probabilities[i] = calculate_probability(other_cell_west, cur_west) * calculate_probability(other_cell_north, cur_north) * calculate_probability(other_cell_east, cur_east)

    # Calculate and normalize probabilities
    total_prob_sum = sum(cell_probabilities) + current_cell_prob
    for i in range(16):
        cell_probabilities[i] = round(cell_probabilities[i] / total_prob_sum, 5)
    
    norm = current_cell_prob / total_prob_sum

    # Output the normalized probabilities
    print(f"P(Move): {norm:.5f} - Moving to cell: {next_cell}")
    print("P(Current Cell):", round(norm, 5))
    print("P(Leftover Cells):", cell_probabilities)

    return next_cell




if __name__ == "__main__":

    # Calculate the starting index based on the robot's initial x and y positions.
    get_index = index_calc(start_x, start_y)
    print("X:", start_x, "Y:", start_y)


    while robot.experiment_supervisor.step(robot.timestep) != -1:
        printMaze(maze)

        if robot.get_lidar_range_image()[400] > 1.0:
            move_forward(1.0, 10.0)
        elif robot.get_lidar_range_image()[200] > 1.0:
            rotate_angle(math.pi + math.pi/1.5, 3)
            adjust_robot_orientation()
            move_forward(1.0, 10.0)
        elif robot.get_lidar_range_image()[600] > 1.0:
            rotate_angle(math.pi/1.5, 3)
            adjust_robot_orientation()
            move_forward(1.0, 10.0)
        else:
            rotate_angle(math.pi + math.pi/2, 3)
            adjust_robot_orientation()
            move_forward(1.0, 10.0)
        calculate_new_position()
        print("************************************")
        print("************************************")
        # Update the current index based on the robot's new position.
        get_index = get_probability(get_index)
