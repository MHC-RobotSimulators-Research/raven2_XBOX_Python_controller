#!/usr/bin/env python
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
import ambf_raven_def as ard
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
# initialize x,y,z for both arms <-- should be inside of move function so it gets sets to 0 at every loop
# x = [0.0,0.0]
# y = [0.0,0.0]
# z = [0.0,0.0]

# initialize grasper angles for both arms
gangle = [0.0,0.0]
# DH values for home position of each arm
home_dh = HOME_DH
def move():
    dis = np.zeros(7)
    # Find safe increment
    safe_increment = int(max(r2py_ctl_l.calc_increment(), r2py_ctl_r.calc_increment()))
    # safe_increment = int(r2py_ctl_l.calc_increment())


    if safe_increment <= man_steps:
        increments = man_steps
    else:
        increments = safe_increment
    print("inc:", increments)

    distl = r2py_ctl_l.countDistance()
    distr = r2py_ctl_r.countDistance()

    scale = 1 / increments
    jrl = scale* distl
    jrr = scale* distr

    for i in range(increments):
        # distl = r2py_ctl_l.countDistance()
        # distr = r2py_ctl_r.countDista
        # nce()
        # scale = min(1.0*i/increments,1.0)
        # jrl = scale* distl
        # jrr = scale* distr
        r2py_ctl_l.pub_jr_command(r2py_ctl_l.seven2sixteen(jrl))
        #r2py_ctl_l.set_jp(jrl)
        r2py_ctl_r.pub_jr_command(r2py_ctl_r.seven2sixteen(jrr))
        #r2py_ctl_r.set_jp(jrr)
    return
# arm_control is a list of boolean: 0 is arm left, 1 is arm right 
# while working==1 and count_interval_cmd<=2:
while working==1:
    
    x = [0.0,0.0]
    y = [0.0,0.0]
    z = [0.0,0.0]
    
    controller = xbox.read()
    print("controller: ", controller)
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

    count_interval_cmd += 1
    
    # Set which control mode to use
    if controller[2][4] and controller[2][5]:
        arm_control[0] = True
        arm_control[1] = True
    elif controller[2][4]:
        arm_control[0] = True
        arm_control[1] = False
    elif controller[2][5]:
        arm_control[0] = False
        arm_control[1] = True
    
    # mod 1: 2 arms controller
    if arm_control[0] and arm_control[1]:
        # update coordinates for left arm, x and y are swaped for more intuitive
        if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
            z[0] = -controller[0][1] / div
            # print("z from controller ly, lt, lb: ", z[0])
        else: 
            # print("controller lx testing: ", controller[0][0])
            if dead_zone< abs(controller[0][0]):
                y[0] = -controller[0][0] / div
                # print("y from controller lx: ", y[0])

            if dead_zone < abs(controller[0][1]):
                x[0] = -controller[0][1] / div
                # print("x from controller ly: ", x[0])                

        # update coordinates for right arm, x and y are swaped
        if controller[1][3] == 1 and dead_zone < abs (controller[1][1]):
            z[1] = - controller[1][1] / div
        else:
            if dead_zone < abs(controller[1][0]):
                y[1] = -controller[1][0] / div
            if dead_zone < abs(controller[1][1]):
                x[1] = -controller[1][1] / div

        print("coords: ", x, y, z)

        # Set gripper angles
        gangle[0] = 1 - (controller[0][2]/4) #ard.HOME_JOINTS[5]+ ard.HOME_JOINTS[6]
        # grasper right angle is opposite --> negative
        gangle[1] = 1 - (controller[1][2]/4) #ard.HOME_JOINTS[5] +ard.HOME_JOINTS[6] 

        r2py_ctl_l.manual_move(0, x[0], y[0], z[0], gangle[0], True, home_dh= HOME_DH)
        r2py_ctl_r.manual_move(1, x[1], y[1], z[1], gangle[1], True, home_dh= HOME_DH)
        move()

    # mod 2: fine control of one arm
    elif arm_control[0] or arm_control[1]:

        # Decide which arm to control
        arm = 0 # default: left arm
        if arm_control[1]:
            arm = 1 # click Back to switch to right arm
        
        # Cartesian control of desired arm
        if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
            z[arm] = -controller[0][1] / div
        else:
            if dead_zone < abs(controller[0][0]):
                y[arm] = -controller[0][0] / div
            if dead_zone < abs(controller[0][1]):
                x[arm] = -controller[0][1] / div
            
        # Set gripper angle
        if arm_control[0]:
            gangle[0] = 1 - (controller[0][2] / 4)
        else:
            gangle[1] = -1 + (controller[1][2] / 4)
        
        # Controller gripper position using home_dh values
        if dead_zone < abs(controller[1][0]) or dead_zone < abs(controller[1][1]):
            # Position j5
            j5 = m.sqrt(controller[1][0] ** 2 + controller[1][1] **2)
            if controller[1][0] < 0:
                home_dh[arm][4] = j5
            else:
                home_dh[arm][4] = -j5
            
            # Position j4
            j4 = m.atan(controller[1][1] / controller[1][0])
            home_dh[arm][3] = j4

        # Plan new position based off of desired cartesian changes

        r2py_ctl_l.manual_move(0, x[0], y[0], z[0], gangle[0], False, home_dh= HOME_DH)
        r2py_ctl_r.manual_move(1, x[1], y[1], z[1], gangle[1], False, home_dh= HOME_DH)
        move()
    

termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)

# rumble the controller when raven is limited
# note: optional, will check when finish core moving functions
# def rumble (arm):
#     rumble = [0.0, 0.0]
#     for i in range(2):
#         if arm.limited[i]:
#             rumble[i] = 1
# if rumble[0] != 0.0 or rumble[1] != 0.0:
#     xbox.rumble(rumble[0], rumble[1], 100)