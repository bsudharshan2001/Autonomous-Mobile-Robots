# # """Lab4_Task1 controller."""

# # #Done by Sudharshan Balaji - U15977125 for Lab 4 of Autonomous Mobile Robots
# # #--------------------------------------------------------------------------
# # # Relative Path - WebotsSim/controllers/Lab4_Task1/Lab4_Task1.py

# # # Landmark Localization

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
robot.load_environment(maze_file[0])
robot.move_to_start()



n_visited = ['.' for _ in range(16)]
cells = [(row, col) for row in range(4) for col in range(4)]



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

        

def find_empty_cell_move(row, column):
    empty_cell_idx = next((idx for idx, cell in enumerate(n_visited) if cell == '.'), None)
    print("\nIdentified target empty cell at index:", empty_cell_idx)

    current_index = cells.index((row, column))
    direction = -1 if empty_cell_idx < current_index else 1

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        print("Now in Index :  ", current_index)

        if current_index == empty_cell_idx:
            print("Robot has arrived at the target empty cell.") 
            return

        target_orientation = (85, 95) if direction == -1 else (265, 275)
        while robot.get_compass_reading() not in range(*target_orientation) and \
                robot.experiment_supervisor.step(robot.timestep) != -1:
            rotate_robot()

        if ((empty_cell_idx // 4 != current_index // 4) or  # Different rows
                (empty_cell_idx % 4 == 0 and direction == 1) or  # Edge cases for row transition
                (current_index % 4 == 0 and direction == -1)):
            move_forward(1.0, 5)
            current_index += 4 * direction
        else:  
            # Adjust orientation for lateral movement within the same row
            rotate_angle(-math.pi/4, 3)
            adjust_robot_orientation()
            move_forward(1.0, 5)
            current_index += direction



def update_visited_status(cell_position, row_position, col_position):
    visit_indicator = 0 

    # Evaluate if the current cell was visited before based on its marker
    if n_visited[cell_position] == 'X':
        print("This cell has been previously marked.")
        visit_indicator = 1

        print("Initiating search for an unmarked cell...")
        find_empty_cell_move(row_position, col_position) 

    else:
        # Update the cell's status to 'X' to indicate it has been visited
        n_visited[cell_position] = 'X'
        formatted_grid = 'Grid Status:\n' 

        for position, status in enumerate(n_visited):
            if position % 4 == 0 and position != 0:
                formatted_grid += '\n'  
            formatted_grid += status + ' '

        print(formatted_grid)

    return visit_indicator


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


    if row is None or column is None:
        # print(f"Warning: Unable to determine cell for x={x}, y={y}. Using fallback values.")
        row = row if row is not None else 0  
        column = column if column is not None else 0

    return row, column


def trilateration(x1, y1, r1, x2, y2, r2, x3, y3, r3):
    """
    Calculates the (x, y) coordinates of the robot based on the distances to three known points.
    This function uses the trilateration method to determine the robot's position in the maze.
    """
    A = 2*x2 - 2*x1
    B = 2*y2 - 2*y1
    C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
    D = 2*x3 - 2*x2
    E = 2*y3 - 2*y2
    F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
    x = (C*E - F*B) / (E*A - B*D)
    y = (C*D - A*F) / (B*D - A*E)
    return x, y



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




if __name__ == "__main__":
    landmark_radius= 0.314
    red_dist, green_dist, blue_dist = None, None, None
    red_seen, green_seen, blue_seen = False, False, False

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        detected_objs = robot.rgb_camera.getRecognitionObjects()
        rotate_robot()

        # Proceed if objects are detected
        for obj in detected_objs:
            colors = obj.getColors()
            position = obj.getPosition()

            if colors[0] == 1 and colors[1] == 1:
                continue

            # Assign distances based on object color
            if colors[0] == 1:  # Red
                red_dist = position[0] + landmark_radius
                red_seen = True
            elif colors[1] == 1:  # Green
                green_dist = position[0] + landmark_radius
                green_seen = True
            elif colors[2] == 1:  # Blue
                blue_dist = position[0] + landmark_radius
                blue_seen = True

            # Check if all colors have been detected
            if red_seen and green_seen and blue_seen:
                print("--------------------------------")
                print("All colors detected. Adjusting orientation.")
                adjust_robot_orientation()

                # Compute position using trilateration
                x, y = trilateration(2, 2, red_dist, -2, -2, green_dist, 2, -2, blue_dist)
                row, col = index_calc(x, y)
                cell_idx = cells.index((row, col))

                # Check if the cell has been visited
                visited = update_visited_status(cell_idx, row, col)
                if not visited:
                    print(f"Location: x={x:.2f}, y={y:.2f}, Index={cell_idx}, Orientation={robot.get_compass_reading()}")

                # Reset detection flags and distances
                red_dist, green_dist, blue_dist = None, None, None
                red_seen, green_seen, blue_seen = False, False, False

                # Handle obstacle detection
                obstacle_dist = robot.get_lidar_range_image()[400]
                if obstacle_dist < 1.0:
                    print("Obstacle detected! Reorienting.")
                    rotate_angle(-math.pi / 2, 10)
                    adjust_robot_orientation()

                # Move to the next cell if it hasn't been visited
                if not visited:
                    print("Proceeding to the next cell.")
                    move_forward(1.0, 5)