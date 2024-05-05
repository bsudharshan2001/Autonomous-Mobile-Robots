"""Lab3_Task1 controller."""

#Done by Sudharshan Balaji - U15977125 for Lab 3 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab3_Task1/Lab3_Task1.py


import os
os.chdir("../..")

from WebotsSim.libraries.MyRobot import MyRobot

robot = MyRobot()

# Loads the environment from the maze file
maze_file = [
    'worlds/mazes/Labs/Lab3/Lab3_Task1.xml',
    'worlds/mazes/Labs/Lab3/Lab3_Task2_1.xml',
    'worlds/mazes/Labs/Lab3/Lab3_Task2_2.xml',
]
robot.load_environment(maze_file[0])
robot.move_to_start()

# max_velocity = robot.max_motor_velocity

def forward_saturation(v):
    max_v= robot.max_motor_velocity

    if v >= max_v:
        v = max_v
    elif v <= -max_v:
        v = -max_v

    return v

def forward_pid(D_maintain, k_p):
    fd = min(robot.get_lidar_range_image()[300:500])
    error = fd - D_maintain
    return k_p * error

def move_to_object():
    """
    This function navigates the robot towards the target object using PID control for forward movement 
    and saturation function
    """

    # Fetch recognition objects from the robot's camera
    detected_objects = robot.rgb_camera.getRecognitionObjects()

    D_maintain = 0.1
    k_p = 10.0

    v_f = forward_pid(D_maintain, k_p)

    v = forward_saturation(v_f) / 2
    # Check if any object is detected
    if detected_objects:
        # Focus on the first detected object as the target
        target_object = detected_objects[0]  # Assuming the first detected object is the target

        # Log target position details
        log_position(target_object)
        print(f"Target in Sight, Moving to it with Current Speed: {v:.2f}")

        # Adjust robot movement if target is within central vision range
        if -0.5 < target_object.getPosition()[1] < 0.5:
            adjust_robot_movement(v, target_object)

def adjust_robot_movement(v, target):
    """
    Adjusts the robot's movement based on the detected target's position, slowing down as it approaches
    the target and eventually stopping when close enough.
    """
    robot.set_left_motors_velocity(v)
    robot.set_right_motors_velocity(v)

    # Stop the robot when it is close to the target
    if target.getPosition()[0] < 0.5:
        print(f"// Robot Stopped at: {target.getPosition()[0]:.2f} meters")
        robot.stop()

def log_position(target):
    """
    Just prints the 
    """
    pos_x, pos_y, pos_z = round(target.getPosition()[0], 2), round(target.getPosition()[1], 2), round(target.getPosition()[2], 2)
    print(f"Robot's Position w.r.t object - X: {pos_x}, Y: {pos_y}, theta: {pos_z}")


if __name__ == "__main__":
    """
    The main execution block runs a loop that controls the robot's actions based on the presence or absence of
    target objects within its field of view. It uses a basic state machine with two states:
    1. Searching for the target by rotating.
    2. Navigating towards the target when detected.
    """

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        current_objects = robot.rgb_camera.getRecognitionObjects()
        print("************************************************************")
        print("************************************************************")
        # State 1: If no object is detected, the robot rotates to search for the target
        if not current_objects:
            print("// Target not found, Reorienting...")
            robot.set_left_motors_velocity(-10)
            robot.set_right_motors_velocity(10)
        # State 2: Object detected, initiate navigation towards the target
        move_to_object()

