"""Lab3_Task2 controller."""

#Done by Sudharshan Balaji - U15977125 for Lab 3 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab3_Task2/Lab3_Task2.py


import os
os.chdir("../..")

from WebotsSim.libraries.MyRobot import MyRobot

robot = MyRobot()

maze_file = [
    'worlds/mazes/Labs/Lab3/Lab3_Task1.xml',
    'worlds/mazes/Labs/Lab3/Lab3_Task2_1.xml',
    'worlds/mazes/Labs/Lab3/Lab3_Task2_2.xml',
]
# robot.load_environment(maze_file[1])
robot.load_environment(maze_file[2])

robot.move_to_start()

max_velocity = robot.max_motor_velocity

# Also included these functions in MyRobot.py
# So instead of using each and every time, we can just call that function
# I have some slight tweaks here for this task, so have defined it again here
def forward_saturation(v):
    max_v= robot.max_motor_velocity

    if v >= max_v:
        v = max_v
    elif v <= -max_v:
        v = -max_v

    return v

def forward_pid(D_maintain, k_p, wall="default"):
    if wall == "R":
        fd = min(robot.get_lidar_range_image()[400:600])
    elif wall == "L":
        fd = min(robot.get_lidar_range_image()[200:400])
    else:   
        fd = min(robot.get_lidar_range_image()[300:500])
    error = fd - D_maintain
    return k_p * error

def wall_follow_pid(wall, D_min=0.50, k_p= 5):
    v_f = forward_pid(0.1, 10, wall)

    rd = min(robot.get_lidar_range_image()[500:600])
    ld = min(robot.get_lidar_range_image()[200:300])

    if wall == "R":
        error = D_min - rd
        if rd < D_min:
            v_r = forward_saturation(v_f)
            v_l = forward_saturation(v_f - abs(k_p * error))

        elif rd > D_min:
            v_r = forward_saturation(v_f - abs(k_p * error))
            v_l = forward_saturation(v_f)
        else:
            v_l = forward_saturation(v_f)
            v_r = forward_saturation(v_f)

    else:
        error = D_min - ld
        if ld < D_min:
            v_r = forward_saturation(v_f - abs(k_p * error))
            v_l = forward_saturation(v_f)

        elif ld > D_min:
            v_r = forward_saturation(v_f)
            v_l = forward_saturation(v_f - abs(k_p * error))

        else:
            v_l = forward_saturation(v_f)
            v_r = forward_saturation(v_f)

    return v_l, v_r

def move_to_object():
    """
    This function navigates the robot towards the target object using PID control for forward movement 
    and saturation function
    """

    # Fetch recognition objects from the robot's camera
    detected_objects = robot.rgb_camera.getRecognitionObjects()

    D_maintain = 0.5
    k_p = 5.0

    v_f = forward_pid(D_maintain, k_p)

    v = forward_saturation(v_f)
    # Check if any object is detected
    if detected_objects:
        # Focus on the first detected object as the target
        target_object = detected_objects[0]  # First detected object is the target

        # Log target position details
        log_position(target_object)

        # Adjust robot movement if target is within central vision range
        if -0.5 < target_object.getPosition()[1] < 0.5:
            adjust_robot_movement(v, target_object)

def adjust_robot_movement(v, target):
    """
    Adjusts the robot's movement based on the detected target's position, slowing down as it approaches
    the target and eventually stopping when close enough.
    """
    print(f"Target in Sight, Moving to it with Current Speed: {v:.2f}")
    robot.set_left_motors_velocity(v)
    robot.set_right_motors_velocity(v)

    # Stop the robot when it is close to the target
    if target.getPosition()[0] < 0.5:
        print(f"// Robot Stopped at: {target.getPosition()[0]:.2f} meters")
        robot.stop()
    elif target.getPosition()[0] < 3.0:
        print(f"Current Speed: {v:.2f}")
        robot.set_left_motors_velocity(v)
        robot.set_right_motors_velocity(v)
    else:
        print(f"Target in Sight, Moving to it with Current Speed: {v:.2f}")
        robot.set_left_motors_velocity(15)
        robot.set_right_motors_velocity(15)


def log_position(target):
    """
    Just prints the position of the robot w.r.t the object
    """
    pos_x, pos_y, pos_z = round(target.getPosition()[0], 2), round(target.getPosition()[1], 2), round(target.getPosition()[2], 2)
    print(f"Robot's Position w.r.t object - X: {pos_x}, Y: {pos_y}, z: {pos_z}")

def rotate_robot(wall):
    if wall == "R":
        robot.set_left_motors_velocity(-15)
        robot.set_right_motors_velocity(15)
    else:
        robot.set_left_motors_velocity(15)
        robot.set_right_motors_velocity(-15)

if __name__ == "__main__":

    while robot.experiment_supervisor.step(robot.timestep) != -1:

        wall = "L"
        D_min=0.5
        k_p=10
        current_objects = robot.rgb_camera.getRecognitionObjects()
        fd = robot.get_lidar_range_image()[400]
        
        print(f"Front Distance = {fd:.2f}m")

        # Determine robot's next action based on its surroundings and detected objects
        if not current_objects:
            print("************************************************************")
            print("----------------- Searching -----------------")

            # Rotate or follow wall based on front obstacle detection
            if fd < 4.0:
                rotate_robot(wall)

            # Follow wall if no front obstacle detected
            else:
                v_l, v_r = wall_follow_pid(wall, D_min, k_p)
                print(f"Adjusting Speed - Left: {v_l:.2f}m/s, Right: {v_r:.2f}m/s")                
                robot.set_left_motors_velocity(v_l)
                robot.set_right_motors_velocity(v_r)

        # Move to object if detected
        else:
            print("************************************************************")
            print("Target Object Found! Moving to it...")
            move_to_object()