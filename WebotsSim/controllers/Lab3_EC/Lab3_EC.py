"""Lab3_EC controller."""

#Done by Sudharshan Balaji - U15977125 for Lab 3 of Autonomous Mobile Robots
#--------------------------------------------------------------------------
# Relative Path - WebotsSim/controllers/Lab3_EC/Lab3_EC.py


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
robot.load_environment(maze_file[1])

robot.move_to_start()

max_velocity = robot.max_motor_velocity

def saturation_func(signal):
    # test for saturation motor velocity
    if signal >= max_velocity:
        signal = max_velocity

    elif signal <= -max_velocity:
        signal = -max_velocity

    return signal
def proportional_gain(d_maintain, kp, wall_to_follow="NONE"):
    # if this function is being used for wall follow, get corresponding front sensor range
    if wall_to_follow == "R":
        d_front = min(robot.get_lidar_range_image()[400:600])
    elif wall_to_follow == "L":
        d_front = min(robot.get_lidar_range_image()[200:400])
    else:
        # if not wall following, range is straight
        d_front = min(robot.get_lidar_range_image()[300:500])

    # print("---------------------------------")
    # print("d_front: ", d_front)

    dist_error_front = d_front - d_maintain
    # print("error_front: ", dist_error_front)
    v_front = kp * dist_error_front

    return v_front

def wall_following_pid(wall_to_follow, d_mid=0.40, k_p=5.0):
    forward_velocity = proportional_gain(0.10, 10.0, wall_to_follow)

    # get sensor readings to detect min distance to side walls
    d_left = min(robot.get_lidar_range_image()[200:300])
    # print("d_left ", d_left)
    d_right = min(robot.get_lidar_range_image()[500:600])
    # print("d_right: ", d_right)

    # perform wall following
    if wall_to_follow == "R":
        error = d_mid - d_right
        if d_right < d_mid:
            v_left = saturation_func(forward_velocity - abs(k_p * error))
            v_right = saturation_func(forward_velocity)

        elif d_right > d_mid:
            v_left = saturation_func(forward_velocity)
            v_right = saturation_func(forward_velocity - abs(k_p * error))

        else:
            v_left = saturation_func(forward_velocity)
            v_right = saturation_func(forward_velocity)

    else:
        error = d_mid - d_left
        if d_left < d_mid:
            v_right = saturation_func(forward_velocity - abs(k_p * error))
            v_left = saturation_func(forward_velocity)

        elif d_left > d_mid:
            v_right = saturation_func(forward_velocity)
            v_left = saturation_func(forward_velocity - abs(k_p * error))

        else:
            v_left = saturation_func(forward_velocity)
            v_right = saturation_func(forward_velocity)

    return v_left, v_right

def motion_to_goal():
    rec_objects = robot.rgb_camera.getRecognitionObjects()

    speed = proportional_gain(0.1, 10.0)
    saturated_speed = saturation_func(speed)

    # if camera has detected an object
    if len(rec_objects) > 0:
        # extract detected an object
        landmark = rec_objects[0]

        # print positions
        print("x: ", round(landmark.getPosition()[0], 2))
        print("y: ", round(landmark.getPosition()[1], 2))
        print("z: ", round(landmark.getPosition()[2], 2))

        # if object is in the center of the camera +- 0.5 meters
        if 0.5 > landmark.getPosition()[1] > -0.5:
            # move forward
            print("OBJECT DETECTED. MOVING TO GOAL...")
            print("SPEED: ", round(saturated_speed/2, 2))
            robot.set_left_motors_velocity(saturated_speed/2)
            robot.set_right_motors_velocity(saturated_speed/2)

            if landmark.getPosition()[0] < 0.5:
                print("ROBOT STOPPED", round(landmark.getPosition()[0], 2), "METERS FROM GOAL")
                robot.stop()

def detect_obstacle_in_path():
    # This function should use sensor data to determine if an obstacle is directly in the path
    d_front = min(robot.get_lidar_range_image()[300:500])
    if d_front < 0.5:
        return True
    return False

def calculate_tangent_points(obstacle):
    """
    Placeholder for calculating tangent points.
    This function calculates points on the obstacle's boundary that provide optimal paths around the obstacle.

    :param obstacle: Data structure containing obstacle boundary information. This might include positions
                     of the edges, angles, or specific points detected via sensors.
    :return: (left_tangent_point, right_tangent_point) Tuple of points representing the left and right tangents.
    """

    robot_position = get_robot_position()  
    goal_position = get_goal_position()  

    best_left_tangent = None
    best_right_tangent = None
    min_left_angle_difference = float('inf')
    min_right_angle_difference = float('inf')

    for point in obstacle_boundary_points(obstacle):  
        angle_to_goal = calculate_angle(point, goal_position)  
        angle_from_robot = calculate_angle(robot_position, point)  

        angle_difference = angle_to_goal - angle_from_robot
        if angle_difference < 0:
            if abs(angle_difference) < min_left_angle_difference:
                min_left_angle_difference = abs(angle_difference)
                best_left_tangent = point
        else:
            if angle_difference < min_right_angle_difference:
                min_right_angle_difference = angle_difference
                best_right_tangent = point

    return best_left_tangent, best_right_tangent


def follow_boundary_and_select_tangent():
    
    print("Following boundary to find tangent...")
    l_speed, r_speed = wall_following_pid("R", d_mid=0.55, k_p=8)  # Example of right wall following
    robot.set_left_motors_velocity(l_speed)
    robot.set_right_motors_velocity(r_speed)

def tangent_bug_navigation(goal_detected, obstacle_in_path):
    if goal_detected and not obstacle_in_path:
        motion_to_goal()
    elif obstacle_in_path:
        follow_boundary_and_select_tangent()

if __name__ == "__main__":

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        rec_objects = robot.rgb_camera.getRecognitionObjects()
        obstacle_in_path = detect_obstacle_in_path() 
        
        goal_detected = len(rec_objects) > 0

        tangent_bug_navigation(goal_detected, obstacle_in_path)
