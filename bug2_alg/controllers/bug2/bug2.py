# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.

import numpy as np
from initialization import *
import math

# boolean function: Checks ifrobot is on m_line
def is_on_M_line(x, y, threshold=0.1):
    goal_point_X = 0.25
    # print(abs(x - goal_point_X))
    # print("On M_Line: ", abs(x - goal_point_X) < threshold)
    return abs(x-goal_point_X) < threshold

# boolean function: rotates robot if not pointed to M
def align_to_M(heading, theta, threshold, turn_left=False):
    ts = 1  # turning speed
    print("Values: ", heading, theta, turn_left)
    print("Difference: ", heading-theta)
    print("Absolute Value: ",abs(heading - theta) < threshold)
    if turn_left:
        ts *= -1  # turn left
    if abs(heading - theta) < threshold:
        return True
    else:
        
        update_motor_speed(input_omega=[ts/20, -1*ts])
        return False

# returns distance between two given points
def calculate_euclidean_distance(x1, y1, x2, y2):
    print("Euclidean Distance: ", math.sqrt((x1-x2)**2 + (y1-y2)**2))
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

if __name__ == "__main__":
    # initialization of robot
    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    print("Robot Initialized")
    init_robot_state(in_pos=[0,0,0], in_omega=[0,0])
    prev = ""
    
    # try getting position of another object or goal object instead of hardcode
    goal_position = 0.25, -1.5

    # define local variables
    state = 'start'
    robot_speed = 3
    forward_left_speeds = [robot_speed, robot_speed]
    far_from_wall_counter = 0
    close_to_wall_counter = 0
    hit_point = []      # x, y
    leave_point = []    # x, y

    # robot loop
    while robot.step(TIME_STEP) != -1:

        # trash variable for unused sonar. can be taken out of initialization.py
        gps_values, compass_val, trash, encoder_value, ir_value = read_sensors_values()
        # print("Sensor Read Complete")
        front_ir_values = ir_value[0], ir_value[7]
        right_ir_values = ir_value[1], ir_value[2]
        left_ir_values = ir_value[5], ir_value[6]


        update_robot_state()
        # print("Current: ", state)
        # print("Previous: ", prev)

        # checks if on M-line before start. might need to be updated to
        # calculate and save m-line points instead.
        # function also works when if statement is taken out.
        if state == 'start':
            # print("Running start state")
            if(is_on_M_line(gps_values[0],gps_values[1])):
                prev = state 
                state = 'align_robot_heading'

        # checks to see if robot is aligned. if align, start moving
        elif state == 'align_robot_heading':
            # print("Running align robot heading state")
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=180, threshold=0.1) 
            if is_aligned: 
                prev = state
                state = 'move_to_goal'

        # updates robot position, moving along the m-line
        # if at goal, stop; if front finds obstacle, wall follow + save hit points
        # maybe calculate slope for m-line and match for alignment and movement?
        elif state == 'move_to_goal':
            # print("Running move to goal state")
            update_motor_speed(input_omega=[robot_speed, robot_speed])
            difference = abs(front_ir_values[1] - front_ir_values[0])
            # print("Difference: ", difference)
            # print("Front_ir_values: ", front_ir_values[0], " ", front_ir_values[1])
            if ((gps_values[0], gps_values[1], goal_position[0], goal_position[1])<0.5):
                state = 'end'
            elif (front_ir_values[0] + front_ir_values[1]) / 2 > 800:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
                    
            
        # follows perimeter of obstacle.
        elif state == 'wall_following':
            # print("Running wall_following state")
            left_wall = left_ir_values[0] > 80
            front_wall = ((front_ir_values[0] + front_ir_values[1]) / 2) > 80
            right_wall = right_ir_values[1] > 80
            # print("Left Values: ", left_ir_values)
            # print("Front Values: ", front_ir_values)
            # print("Right Values: ", right_ir_values)
            # print("Left Wall: ", left_wall)
            # print("Front Wall: ", front_wall)
            # print("Right Wall: ", right_wall)

            # found wall in front, default turn to left
            if front_wall: 
                # print("Running front wall")
                update_motor_speed(input_omega=[-1*robot_speed, robot_speed])

            # is on m-line after wall following, creates leave point and aligns to goal again
            elif (is_on_M_line(gps_values[0], gps_values[1])) and prev == 'wall_following':
                # print("Running M_line")
                # print(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1]) < calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_postition[0], goal_postition[1]))
                if calculate_euclidean_distance(gps_values[0], gps_values[1], goal_position[0], goal_position[1]) < calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_position[0], goal_position[1]):
                    #print("Running E-Distance")
                    leave_point.append([gps_values[0], gps_values[1]])
                    prev = state
                    state = 'align_robot_heading'

            # following wall on the right
            elif right_wall: 
                update_motor_speed(input_omega=[robot_speed, robot_speed])
                if calculate_euclidean_distance(gps_values[0], gps_values[1], hit_point[-1][0], hit_point[-1][0]) > 1:
                    #print("Prev Changed 1")
                    prev = state

            # if too far from the wall
            else:
                #print("Running else")
                update_motor_speed(input_omega=[robot_speed, robot_speed/4])
                #print("Prev Changed 2")
                #prev = state

        elif state == 'end':
            print("Running end state")
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=90, threshold = 0.1)
            if is_aligned:
                update_motor_speed(input_omega=[0, 0, 0]) #end
                
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_position[0], goal_position[1])< 0.1):
            state = 'end'
            #print("State changed to end")
            update_motor_speed(input_omega=[0, 0, 0])
        
    pass
           
