import raven_py_xbox_controller
from xbox_controller import xbox_controller
from ambf_raven_def import *
"""
Raven 2 Control - Control software for the Raven II robot
Copyright (C) 2005-2022  Andrew Lewis, Yun-Hsuan Su, Haonan Peng, Blake Hannaford,
and the University of Washington BioRobotics Laboratory

This file is part of Raven 2 Control.

Raven 2 Control is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Raven 2 Control is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.



raven_keyboard_controller.py

Python controller for RAVEN II using CRTK API

date Jan 10, 2022
author Haonan Peng, Yun-Hsuan Su, Andrew Lewis, 
"""

import tty, sys, termios
import time
import utils_r2_keyboard_controller as utils
import rospy
import numpy as np
import raven_py_xbox_controller
import pandas as pd
import threading as th

command_interval = 0.2 # [IMPT]: 0,2 is usually good, increase this factor will cause a larger control delay, but may increase the publishing rate

velocity_joint_1 = 3 # degree/s
velocity_joint_2 = 3 # degree/s
velocity_joint_3 = 0.01 # m/s
velocity_joint_4 = 10 # degree/s
velocity_joint_5 = 10 # degree/s
velocity_joint_6 = 10 # degree/s
velocity_joint_7 = 10 # degree/s

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

filedescriptors = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)

rospy.init_node('raven_keyboard_controller', anonymous=True)

r2py_ctl_l = raven_py_xbox_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm1', grasper_name = 'grasp1')
r2py_ctl_r = raven_py_xbox_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm2', grasper_name = 'grasp2')
r2py_ctl_l.pub_state_command('resume')
r2py_ctl_r.pub_state_command('resume')


x = 0
working = 1
utils.print_manu()

time_last_key_command = 0
count_interval_cmd = 0

xbox = xbox_controller()
time.sleep(5)
arm_control = [True, True]
# how much the raw input values will be divided into x,y,z
div = 500 # may change later if too slow
dead_zone = 0.1

#man_steps are the steps to divide the distance and increment (man_steps)steps to get to the goal lpos
man_steps = 17
# True for p5 ik and false for standard ik
ik_mode = True

# initialize grasper angles for both arms
gangle = [0.0,0.0]
# DH values for home position of each arm
home_dh = HOME_DH

# create pandas dataframe to record xbox controller command 
controller_df = pd.DataFrame(columns = ['LeftJoystickX', "LeftJoystickY", "LeftTrigger", "LeftBumper", "RightJoystickX", "RightJoystickY", "RightTrigger", "RightBumper", "A", "B", "X", "Y", "Back", "Start"])

# initialize csv controll mode
csv_controller = False

# initialize index in the csv
i = 0

def get_input():
    global working 
    user_input = sys.stdin.read(1)[0]
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)
    if user_input == 'q':
        working = 0
    elif user_input == "s":
        working = 1
    elif user_input == "h":
        working = 2

def move_home():
    r2py_ctl_l.set_home()
    r2py_ctl_r.set_home()
    move()
    
def move():
    """
    Uses the helper function move increment to move to the next planned position
    over a number of steps equal to man_steps or the number of increments required
    to keep each move within safe limits
    """
    dis = np.zeros(7)
    # Find safe increment
    safe_increment = int(max(r2py_ctl_l.calc_increment(), r2py_ctl_r.calc_increment()))
    # safe_increment = int(r2py_ctl_l.calc_increment())

    if safe_increment <= man_steps:
        increments = man_steps
    else:
        increments = safe_increment
    # print("inc:", increments)

    distl = r2py_ctl_l.countDistance()
    distr = r2py_ctl_r.countDistance()

    scale = 1 / increments
    jrl = scale* distl
    jrr = scale* distr

    for i in range(increments):    
        r2py_ctl_l.pub_jr_command(r2py_ctl_l.seven2sixteen(jrl))
        #r2py_ctl_l.set_jp(jrl)
        r2py_ctl_r.pub_jr_command(r2py_ctl_r.seven2sixteen(jrr))
        #r2py_ctl_r.set_jp(jrr)
        time.sleep(0.003)
    return

def update_pos_two_arm(dead_zone, controller, div):

    # Coarse control of both raven arms
    pos = [[0.0, 0.0],  # x - relative
           [0.0, 0.0],  # y - relative
           [0.0, 0.0],  # z - relative
           [0.0, 0.0]]  # gangle - absolute
    # Update coordinates for left arm, note x and y are swapped to make controls more intuitive

    # update z 
    if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
        pos[2][0] = -controller[0][1] / div
    else:
        # update y
        if dead_zone < abs(controller[0][0]):
            pos[1][0] = -controller[0][0] / div
        # update x
        if dead_zone < abs(controller[0][1]):
            pos[0][0] = controller[0][1] / div

    # Update coordinates for right arm
    # update z
    if controller[1][3] == 1 and dead_zone < abs(controller[1][1]):
        pos[2][1] = -controller[1][1] / div
    else:
        # update y
        if dead_zone < abs(controller[1][0]):
            pos[1][1] = -controller[1][0] / div
        # update x
        if dead_zone < abs(controller[1][1]):
            pos[0][1] = controller[1][1] / div
    # Set gripper angles
    pos[3][0] = 1 - (controller[0][2] / 4) #ard.HOME_JOINTS[5]+ ard.HOME_JOINTS[6]
    # for the right arm gangle needs to be negative, this is to fix a bug somewhere else that I can't find
    pos[3][1] = 1 - (controller[1][2] / 4) #ard.HOME_JOINTS[5] +ard.HOME_JOINTS[6] 
    return pos

def update_pos_one_arm(dead_zone, controller, div, arm, home_dh):

    pos = [[0.0, 0.0],  # x - relative
           [0.0, 0.0],  # y - relative
           [0.0, 0.0],  # z - relative
           [0.0, 0.0]]  # gangle - absolute

    # Cartesian control of desired arm
    if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
        pos[2][arm] = -controller[0][1] / div
    else:
        if dead_zone < abs(controller[0][0]):
            pos[1][arm] = -controller[0][0] / div
        if dead_zone < abs(controller[0][1]):
            pos[0][arm] = -controller[0][1] / div

    # Left arm
    if not arm:
        # Set left gripper angle
        pos[3][0] = 1 - (controller[1][2] / 4)

        # Set right j4
        if dead_zone < abs(controller[1][0]):
            if abs(home_dh[0][3] - controller[1][0] / 10) < m.pi:
                home_dh[0][3] += -controller[1][0] / 10
        # Position j5
        if dead_zone < abs(controller[1][1]):
            if abs(home_dh[0][4] - controller[1][1] / 10) < 2:
                home_dh[0][4] += -controller[1][1] / 10

    # Right arm
    else:
        # Set right gripper angle
        pos[3][1] = 1 - (controller[1][2] / 4)

        # Set right j4
        if dead_zone < abs(controller[1][0]):
            if abs(home_dh[1][3] + controller[1][0] / 10) < m.pi:
                home_dh[1][3] += controller[1][0] / 10
        # Position j5
        if dead_zone < abs(controller[1][1]):
            if abs(home_dh[1][4] + controller[1][1] / 10) < 2:
                home_dh[1][4] += controller[1][1] / 10

    return pos, home_dh

# get_inputs = th.Thread(target = get_input, args = ())
# get_inputs.start()

# arm_control is a list of boolean: 0 is arm left, 1 is arm right 
# while working==1 and count_interval_cmd<=2:

while working == 1:
    if not csv_controller:
        controller = xbox.read()
    
    elif csv_controller:
        row = xbox.csv_reader()
        if row!= None:
            controller = row
        else:
            break
    # Use the loc method to add the new row to the DataFrame
    controller_row = [controller[0][0], controller[0][1], controller[0][2], controller[0][3], controller[1][0], controller[1][1], controller[1][2], controller[1][3], controller[2][0], controller[2][1], controller[2][2], controller[2][3], controller[2][4], controller[2][5]]
    controller_df.loc[len(controller_df)] = controller_row
    
    '''
    Manual control mode for the simulated raven2 using an xbox controller. There are two
    modes. The first enables simultaneous control of both arms on the xyz axes, but locks
    joints 4, 5, and 6 to their home positions. Accessed by simultaneously pressing back 
    and start buttons.
    
    Left stick: left arm x and y
    Left trigger: left arm gripper open close
    Left button: when pressed left stick up and down controls left arm z
    Right stick: right arm x and y
    Right trigger: right arm gripper open close
    Right button: when pressed right stick up and down controls right arm z
    The second control mode only controls one arm at a time, but adds control of joints 4 and 5.
    Accessed by pressing back for the left arm and start for the right arm.
    
    Left stick: selected arm x and y
    Left trigger: selected arm gripper open close
    Left button: when pressed left stick up and down controls selected arm z
    Right stick: controls grippers angle and rotation
    '''
    
    # if count_interval_cmd >= 100: 
    #     input_key = sys.stdin.read(1)[0]
    #     termios.tcflush(sys.stdin, termios.TCIOFLUSH)
    #     if input_key == '9':
    #         r2py_ctl_l.pub_state_command('pause')
    #         sys.exit('Closing RAVEN 2 XBox controller')
    #     count_interval_cmd = 0
    # count_interval_cmd += 1
    print(controller)
    count_interval_cmd += 1
    
    # Set which control mode to use
    if controller[2][4] and controller[2][5]:
        arm_control[0] = True
        arm_control[1] = True
        print("Controlling both arms")
    elif controller[2][4]:
        arm_control[0] = True
        arm_control[1] = False
        print("Controlling the left arm")
    elif controller[2][5]:
        arm_control[0] = False
        arm_control[1] = True
        print("Controlling the right arm")

    # Set kinematics mode
    if controller[2][0]:
        ik_mode = True
        print("Using p5 inverse kinematics")
    elif controller[2][1]:
        ik_mode = False
        print("Using standard inverse kinematics")
    # print(ik_mode)

    print(arm_control)
    # mod 1: 2 arms controller
    if arm_control[0] and arm_control[1]:
        # modify position using controller inputs
        pos = update_pos_two_arm(dead_zone, controller, div)
        r2py_ctl_l.plan_move(0, pos[0][0], pos[1][0], pos[2][0], pos[3][0], True, home_dh)
        r2py_ctl_r.plan_move(1, pos[0][1], pos[1][1], pos[2][1], pos[3][1], True, home_dh)
        move()

    # mod 2: fine control of one arm
    elif arm_control[0] or arm_control[1]:
        # Decide which arm to control
        arm = 0
        if arm_control[1]:
            arm = 1
        # modify position using controller inputs
        pos, home_dh = update_pos_one_arm(dead_zone, controller, div, arm, home_dh)
        # Plan new position based off of desired cartesian changes
        if arm == 0:
            r2py_ctl_l.plan_move(arm, pos[0][arm], pos[1][arm], pos[2][arm], pos[3][arm], True, home_dh)
        elif arm == 1:
            r2py_ctl_r.plan_move(arm, pos[0][arm], pos[1][arm], pos[2][arm], pos[3][arm], True, home_dh)
        move()
    # time.sleep(0.01)
    # increment i after each time move to save new row for csv file
    i += 1
# while working == 2:
#     # move_home()
#     working = 1
# save the csv 
controller_df.to_csv('controller_csv/controller.csv')
print("saved csv")
termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)
# get_inputs.join()