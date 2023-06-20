import raven_py_controller
import xbox_controller
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
import raven_py_controller

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

r2py_ctl_l = raven_py_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm1', grasper_name = 'grasp1')
r2py_ctl_r = raven_py_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm2', grasper_name = 'grasp2')
r2py_ctl_l.pub_state_command('resume')

x = 0
working = 1
utils.print_manu()

time_last_key_command = 0
count_interval_cmd = 101

xbox = xbox_controller()
buttons = xbox.get_buttons()

while working==1:

    # if time.time() - time_last_key_command >= command_interval:
    #     print('--------------------------------------------------------------------------------')
    #     print('--------------------------------------------------------------------------------')
    #     print('--------------------------------------------------------------------------------')
    #     time_last_key_command = time.time()
    #     input_key = sys.stdin.read(1)[0]

    if count_interval_cmd >= 100: 
        input_key = sys.stdin.read(1)[0]
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)
        if input_key == '9':
            r2py_ctl_l.pub_state_command('pause')
            sys.exit('Closing RAVEN 2 Keyboard controller')
        count_interval_cmd = 0
    count_interval_cmd += 1

    if buttons[0] == 1:
        utils.print_no_newline(" Moving: Joint 1 +++         ")
        cmd = np.zeros((16))
        cmd[1] = velocity_joint_1 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[0] == 0:
        utils.print_no_newline(" Moving: Joint 1 ---         ")
        cmd = np.zeros((16))
        cmd[1] = -velocity_joint_1 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[1] == 1:
        utils.print_no_newline(" Moving: Joint 2 +++         ")
        cmd = np.zeros((16))
        cmd[2] = velocity_joint_2 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)
              
    elif buttons[1] == 0:
        utils.print_no_newline(" Moving: Joint 2 ---         ")
        cmd = np.zeros((16))
        cmd[2] = -velocity_joint_2 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[2] == 1:
        utils.print_no_newline(" Moving: Joint 3 +++         ")
        cmd = np.zeros((16))
        cmd[3] = velocity_joint_3 * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)
        
    elif buttons[2] == 0:
        utils.print_no_newline(" Moving: Joint 3 ---         ")
        cmd = np.zeros((16))
        cmd[3] = -velocity_joint_3 * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[3] == 1:
        utils.print_no_newline(" Moving: Joint 4 +++         ")
        cmd = np.zeros((16))
        cmd[4] = velocity_joint_4 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)
        
    elif buttons[3] == 0:
        utils.print_no_newline(" Moving: Joint 4 ---         ")
        cmd = np.zeros((16))
        cmd[4] = -velocity_joint_4 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[4]== 1:
        utils.print_no_newline(" Moving: Joint 5 +++         ")
        cmd = np.zeros((16))
        cmd[5] = velocity_joint_5 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)
        
    elif buttons[4] == 0:
        utils.print_no_newline(" Moving: Joint 5 ---         ")
        cmd = np.zeros((16))
        cmd[5] = -velocity_joint_5 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[5] == 1:
        utils.print_no_newline(" Moving: Joint 6 +++         ")
        cmd = np.zeros((16))
        cmd[6] = velocity_joint_6 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)
        
    elif buttons[5] == 0:
        utils.print_no_newline(" Moving: Joint 6 ---         ")
        cmd = np.zeros((16))
        cmd[6] = -velocity_joint_6 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[6] == 1:
        utils.print_no_newline(" Moving: Joint 7 +++         ")
        cmd = np.zeros((16))
        cmd[7] = velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)
        
    elif buttons[6] == 0:
        utils.print_no_newline(" Moving: Joint 7 ---         ")
        cmd = np.zeros((16))
        cmd[7] = -velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[9] == 1:
        utils.print_no_newline(" Moving: Grasper Left open         ")
        cmd = np.zeros((16))
        cmd[6] = velocity_joint_6 * Deg2Rad * 1e-3
        cmd[7] = velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl_l.pub_jr_command(cmd)

    elif buttons[9] == 0:
        utils.print_no_newline(" Moving: Grasper Left close         ")
        cmd = np.zeros((16))
        cmd[6] = -velocity_joint_6 * Deg2Rad * 1e-3
        cmd[7] = -velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl_r.pub_jr_command(cmd)
        
    elif buttons[10] == 1:
        utils.print_no_newline(" Moving: Grasper Right open         ")
        cmd = np.zeros((16))
        cmd[14] = velocity_joint_6 * Deg2Rad * 1e-3
        cmd[14] = velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl_r.pub_jr_command(cmd)

    elif buttons[10] == 0:
        utils.print_no_newline(" Moving: Grasper Right close         ")
        cmd = np.zeros((16))
        cmd[15] = -velocity_joint_6 * Deg2Rad * 1e-3
        cmd[15] = -velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl_r.pub_jr_command(cmd)
    # Cartisian space control----------------------------------------------
    r2py_ctl_l.pub_cr_command()
    r2py_ctl_r.pub_cr_command()

termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)


