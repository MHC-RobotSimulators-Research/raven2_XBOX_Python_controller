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



raven_py_controller.py

Python controller for RAVEN II using CRTK API

date Jan 10, 2022
author Haonan Peng, Yun-Hsuan Su, Andrew Lewis, 
"""

import time

import rospy
import numpy as np
# from scipy.spatial.transform import Rotation as sp_rot
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

import crtk_msgs.msg # crtk_msgs/operating_state
import utils_r2_py_controller as utils
from xbox_controller import xbox_controller

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

import raven_fk as fk
import raven_ik as ik 
import ambf_raven_def as ard
class raven2_py_controller():

    # ros node is not initialized here, it should be initialized and named by the program that use this controller
    def __init__(self, name_space, robot_name, grasper_name):
        self.name_space = name_space
        self.robot_name = robot_name
        self.grasper_name = grasper_name

        #self.joint_velocity_factor = np.array([1e-5, 1e-5 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5]) # 1e-5 means target speed is 1.0cm (1e-5 m) per second

        self.max_jr = np.array([5*Deg2Rad, 5*Deg2Rad, 0.02, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad]) #, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad]) # This is the max velocity of jr command, should be rad/sec and m/sec for rotation and translation joints 
        # self.max_jr = np.array([0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005]) #, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad]) # This is the max velocity of jr command, should be rad/sec and m/sec for rotation and translation joints 

        #add max_cr
        self.max_cr = 0.00018
        self.rate_pub = 500 # !IMPT This is the publish rate of the motion command publisher. It must be tested, because it will affect the real time factor and thus affect the speed. If you are not sure or cannot test, use a large rate (such as 1000) can be safer.
        self.rate_pub_cr = 1000
        self.max_rate_move = 500 # This is a protection, if the time interval between 2 move command is shorter than 1/rate, the publisher will wait util 1/rate
        self.min_interval_move = 1.0/self.max_rate_move
        self.pos_scaler = 1000.0/self.rate_pub # This is to scale the pos command to meet the target velocity
        self.time_last_pub_move = -1.0

        self.max_jr = self.max_jr / self.rate_pub
        #max_jr is 0.00017453292
        self.max_cr = self.max_jr / self.rate_pub_cr

        self.operate_state = None # [String] current robot operation state, according the CRTK standard - "DISABLED", "ENABLED", "PAUSED", "FAULT", robot can only be controlled when "ENABLED"
        self.is_homed = None 
        self.is_busy = None

        #self.command_type = 'relative' # can be 'relative' or 'absolute', if 'relative' the command will be sent through 'jr' in CRTK, if 'absolute', the command will be sent through 'jp'

        self.measured_cpos_tranform = np.zeros((4,4)) # np.array 4x4 transform matrix of the end-effector measured position
        self.measured_jpos = None # (15,) array of measured joint position
        self.measured_jvel = None # (15,) array of measured joint velocity
        self.measured_jeff = None # (15,) array of measured joint effort

        self.pub_count_motion = 0 # The counts or how many motion command messages are sent
        # temp_measured_joint: replace measured_jpos when cannot connect with raven computer
        # size 15 to match with measured_joint, when start set it to home_joints
        self.temp_measured_jpos = ard.HOME_JOINTS
        # add xbox controller
        self.cmd = xbox_controller()
        # initualize steps from current increment to new joint positions
        self.man_step = 20
        # new joint position
        self.new_jp = np.zeros(7)
        self.__init_pub_sub()

        #boolean if pass the limit
        self.limited = [False, False]
        return None

    def __del__(self):
        self.pub_state_command('pause') 
        return None

    # setup ros publishers and subscribers
    def __init_pub_sub(self):

        # Subscriber and publisher of robot operation state
        topic = "/" + self.robot_name + "/operating_state"
        self.__subscriber_operating_state = rospy.Subscriber(topic, crtk_msgs.msg.operating_state, self.__callback_operating_state)

        topic = "/" + self.robot_name + "/state_command"
        self.__publisher_state_command = rospy.Publisher(topic, crtk_msgs.msg.StringStamped, latch = True, queue_size = 1)

        # Subscribers of joint state and cartisian position
        topic = "/" + self.robot_name + "/measured_cp"
        self.__subscriber_measured_cp = rospy.Subscriber(topic, geometry_msgs.msg.TransformStamped, self.__callback_measured_cp)

        # topic = "/" + self.robot_name + "/measured_js" # [IMPT] this should be the usual case
        topic = "/arm2/measured_js" # [IMPT] This line is because the RAVEN I use has a mismatch that the arm1's jpos is published on arm2
        self.__subscriber_measured_js = rospy.Subscriber(topic, sensor_msgs.msg.JointState, self.__callback_measured_jp)

        # robot movement publishers
        topic = "/" + self.robot_name + "/servo_cr"
        self.__publisher_servo_cr = rospy.Publisher(topic, geometry_msgs.msg.TransformStamped, latch = True, queue_size = 1)

        topic = "/" + self.robot_name + "/servo_cp"
        self.__publisher_servo_cp = rospy.Publisher(topic, geometry_msgs.msg.TransformStamped, latch = True, queue_size = 1)

        topic = "/" + self.robot_name + "/servo_cv"
        self.__publisher_servo_cv = rospy.Publisher(topic, geometry_msgs.msg.TransformStamped, latch = True, queue_size = 1)

        topic = "/" + self.robot_name + "/servo_jr"
        self.__publisher_servo_jr = rospy.Publisher(topic, sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        topic = "/" + self.robot_name + "/servo_jv"
        self.__publisher_servo_jv = rospy.Publisher(topic, sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        topic = "/" + self.robot_name + "/servo_jp"
        self.__publisher_servo_jp = rospy.Publisher(topic, sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        topic = "/" + self.grasper_name + "/servo_jr"
        self.__publisher_servo_gr = rospy.Publisher(topic, sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        topic = "/" + self.grasper_name + "/servo_jv"
        self.__publisher_servo_gv = rospy.Publisher(topic, sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        topic = "/" + self.grasper_name + "/servo_jp"
        self.__publisher_servo_gp = rospy.Publisher(topic, sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        return None
    
    """
    getter and setter for temp_measured_jpos 
    for substituting self.measured_jpos when cannot connecting with Raven computer
    """
    def get_temp_measured_jpos(self):
        return self.temp_measured_jpos
    
    def set_jp(self, new_jr):
        self.temp_measured_jpos = new_jr + self.temp_measured_jpos
        return 
    
    def __check_max_jpose_command(self, joint_command):
        max_jr = np.array([5*Deg2Rad, 5*Deg2Rad, 0.02, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad]) / 500 # This is the max velocity of jr command, should be rad/sec and m/sec for rotation and translation joints 

        diff = max_jr - np.abs(joint_command)
        #print("max_jr: ", self.max_jr)
        #print("joint command: ", joint_command)
        #print("joint command",joint_command)
        #joint 2: 0.785398163
        #joint 3: -0.0000000000000000000555111512
        #joint 6: -0.5
        #joint 7: -0.5
        #print("diff: ", diff)
        #joint 1: 0.000174532925
        #joint 2: -0.785223630
        #joint 6: -0.5
        #joint 7: -0.5
        idx = np.array(np.where(diff<0))

        return idx[0]

    def __check_max_cr_command(self,coordinate):
        diff = self.max_cr - np.abs(coordinate)
        idx = np.array(np.where(diff<0))

        return idx[0]
    def __callback_measured_cp(self, msg):
        # rot = sp_rot.from_quat([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

        # self.measured_cpos_tranform[0:3,0:3] = rot.as_matrix() # there is an error, 'no method called as_matrix'
        self.measured_cpos_tranform[0,3] = msg.transform.translation.x
        self.measured_cpos_tranform[1,3] = msg.transform.translation.y
        self.measured_cpos_tranform[2,3] = msg.transform.translation.z
        self.measured_cpos_tranform[3,3] = 1.0

        return None

    def __callback_measured_jp(self, msg):
        self.measured_jpos = np.array(msg.position)
        #print("joint position jp: ", self.measured_jpos)
        self.measured_jvel = np.array(msg.velocity)
        self.measured_jeff = np.array(msg.effort)
        return None

    def __callback_operating_state(self, msg):

        self.operate_state = msg.state

        return None

    # [Input]: state_command - String, including "enable", "disable", "pause", "resume", "unhome", "home", "NULL"
    def pub_state_command(self, state_command):
        if state_command not in ["enable", "disable", "pause", "resume", "unhome", "home", "NULL"]:
            utils.print_ROS_INFO('Unknown state command of RAVEN, can not publish')
            return -1

        msg_state_command = crtk_msgs.msg.StringStamped
        msg_state_command.string = state_command
        #msg_state_command.header.stamp = msg_state_command.header.stamp.now() # used in c++, seems not necessary for Python

        print('Robot state command published, command: ' + msg_state_command.string)

        self.__publisher_state_command.publish(msg_state_command)

        return 0

    # [Return]: self.operate_state - String, one of "DISABLED", "ENABLED", "PAUSED", "FAULT", robot can only be controlled when "ENABLED"
    def get_robot_state(self):

        return  self.operate_state

    # [IMPT]: the joint_command is an np.array of dimension 16, please notice that the first entry is always 0 and does nothing, 
    #         this is to make the command consistent and intuitive - command[1] is joint 1 and [2] is joint 2, so on and so forth.
    #         in the controller code, this only applies to the input joint_command here, nowhere else
    # [Input ]: joint_command - an np.array of dimension 16, joint_command[1] should be the expected velocity of joint 1 (rad and m /sec)
    # [Return]: -1 if command not published, 0 if command published normally
    # [Note]: There is no clear reason why the dimension of the joint command is 15 in CRTK RAVEN. But it is confirmed that this is not to control 2 arms. Each arm should have its own controller node
    def pub_jr_command(self, joint_command):
        # joint_command = np.array([0.0005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005,  0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005, 0.00005]) # This is the max velocity of jr command, should be rad/sec and m/sec for rotation and translation joints 

        joint_command = joint_command[1:] # This is to meet the format of CRTK, where joint 1 is at index 0
        
        max_check = self.__check_max_jpose_command(joint_command)
        if max_check.size != 0:
            print('Command velocity too fast, joints: ')
            #print(max_check)
            print('Command not sent')
            
            return -1
        # print("joint command: ",joint_command)
        msg = sensor_msgs.msg.JointState()
        msg.position[:] = joint_command.flat
        
        max_check = self.__check_max_jpose_command(joint_command)

        interval_pub = time.time() - self.time_last_pub_move
        #print(str(interval_pub)) # [debug]
        if (self.time_last_pub_move != -1.0) & (interval_pub < self.min_interval_move):
            time.sleep(self.min_interval_move-interval_pub) # If the time interval is too short, wait util do not exceed the max rate
            #print('time sleep:' + str(self.min_interval_move-interval_pub)) #[debug]
        self.__publisher_servo_jr.publish(msg)
        #print("command sent!")
        self.time_last_pub_move = time.time()
        self.pub_count_motion += 1

        #print('Command pub count: ' + str(self.pub_count_motion) + ' | msg: ' + str(joint_command)) # [debug]

        return 0
    
    # [IMPT]: the joint_command is an np.array of dimension 16, please notice that the first entry is always 0 and does nothing, 
    #         this is to make the command consistent and intuitive - command[1] is joint 1 and [2] is joint 2, so on and so forth.
    #         in the controller code, this only applies to the input joint_command here, nowhere else
    # [Input ]: target_jpos - in degree an np.array of dimension 16, target_jpos[1] should be the target pose of joint 1 (rad and m /sec) || max_vel in degree, an np.array of dimension 16
    # [Return]: 0 if finished moving
    # [Note]: There is no clear reason why the dimension of the joint command is 15 in CRTK RAVEN. But it is confirmed that this is not to control 2 arms. Each arm should have its own controller node
    def go_to_jr(self, target_jpos, max_vel = np.array([0, 3*Deg2Rad, 3*Deg2Rad, 0.01, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad])):

        target_jpos = target_jpos[1:]  # This is to meet the format of CRTK, where joint 1 is at index 0
        
        target_jpos_1 = target_jpos[0] 
        target_jpos_2 = target_jpos[1] 
        target_jpos_3 = target_jpos[2]
        
        moving = True
        count = 0
        while moving == True:
            cur_jpos_1 = self.measured_jpos[0]
            cur_jpos_2 = self.measured_jpos[1]
            cur_jpos_3 = self.measured_jpos[2]
            
            cmd = np.zeros((16))
            if (np.abs(target_jpos_1 - cur_jpos_1)*Rad2Deg > 1):
                cmd[1] = np.clip(target_jpos_1 - cur_jpos_1, -max_vel[1], max_vel[1])
            if (np.abs(target_jpos_2 - cur_jpos_2)*Rad2Deg > 1):
                cmd[2] = np.clip(target_jpos_2 - cur_jpos_2, -max_vel[2], max_vel[2])
            if (np.abs(target_jpos_3 - cur_jpos_3) > 0.005):
                cmd[3] = np.clip(target_jpos_3 - cur_jpos_3, -max_vel[3], max_vel[3])
                
            self.pub_jr_command(cmd * 1e-3)
            count += 1
            
            if count % 50 == 0:
                print('--------------------------------------------')
                print('Joint 1 target (Deg): ' + str(target_jpos_1*Rad2Deg) + ' | current: '  + str(cur_jpos_1*Rad2Deg)) #
                print('Joint 2 target (Deg): ' + str(target_jpos_2*Rad2Deg) + ' | current: '  + str(cur_jpos_2*Rad2Deg))
                print('Joint 3 target (m): ' + str(target_jpos_3) + ' | current: '  + str(cur_jpos_3))
            
            if (np.abs(target_jpos_1 - cur_jpos_1)*Rad2Deg <= 1) & (np.abs(target_jpos_2 - cur_jpos_2)*Rad2Deg <= 1) & (np.abs(target_jpos_3 - cur_jpos_3) <= 0.005):
                moving = False

            
        return 0
    
    #created by Mai Bui 
    def pub_cr_command(self,x,y,z):

        msg = geometry_msgs.msg.TransformStamped()
        cmd_coor = [x,y,z]
        max_check = self.__check_max_cr_command(cmd_coor)
        if max_check.size != 0:
            print('Command velocity too fast, axis: ')
            for axis in max_check:
                if axis == 0:
                    print("x")
                elif axis == 1:
                    print("y")
                elif axis == 2:
                    print("z")
            print('Command not sent')
            return -1
    
        msg.header.stamp = msg.header.stamp.now()
        t = msg.transform
        t.translation.x = x
        t.translation.y = y
        t.translation.z = z

        interval_pub = time.time() - self.time_last_pub_move
        #print(str(interval_pub)) # [debug]
        if (self.time_last_pub_move != -1.0) & (interval_pub < self.min_interval_move):
            time.sleep(self.min_interval_move-interval_pub)
        self.__publisher_servo_cr.publish(msg)
        self.time_last_pub_move = time.time()
        self.pub_count_motion += 1

        #print('Command pub count: ' + str(self.pub_count_motion) + ' | msg: ' + str(joint_command)) # [debug]

        return 0    
    
        # return a np array sized 7
    def countDistance(self):
        #print('new_jp: ', self.new_jp)
        #print('measured_jpos: ', self.measured_jpos)
        #return self.new_jp - self.measured_jpos
        return self.new_jp - self.get_temp_measured_jpos()

    def calc_increment(self):
        """
        Calculates the difference between the current joint positions and planned joint positions
        then calculates the number of increments required to stay within joint rotation limits
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
        """
        # Calculate delta jp
        max_inc_jr = np.array([5*Deg2Rad, 5*Deg2Rad, 0.02, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad]) / 500 #, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad]) # This is the max velocity of jr command, should be rad/sec and m/sec for rotation and translation joints 
        # Find safe increment
        # increment = self.countDistance() / self.max_jr
        increment = self.countDistance() / max_inc_jr

        #print("increments: ", increment)
        return max(map(abs, increment)) + 1

    def manual_move(self, arm, x, y, z, gangle, p5=False, home_dh=ard.HOME_DH):
        """
        moves the desired robot arm based on inputted changes to cartesian coordinates
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
            x (float) : the desired change to the x coordinate
            y (float) : the desired change to the y coordinate
            z (float) : the desired change to the z coordinate
            gangle (float) : the gripper angle, 0 is closed
            p5 (bool) : when false uses standard kinematics, when true uses p5 kinematics
            home_dh (array) : array containing home position, or desired postion of the
                joints not set by cartesian coordinates in inv_kinematics_p5
        """
        #print("size curr_jp: ", len(self.measured_jpos))
        #curr_jp = self.measured_jpos
        curr_jp = self.get_temp_measured_jpos()
        #curr_jp = np.zeros(7,dtype="float")
        #print("current joint positions: " + str(curr_jp))
        if p5:
            curr_tm = fk.fwd_kinematics_p5(arm, curr_jp)
            print(curr_tm)
        else:
            curr_tm = fk.fwd_kinematics(arm, curr_jp)
        curr_tm[0, 3] += x
        print("x: ",x)
        curr_tm[1, 3] += y
        print("y: ", y)
        curr_tm[2, 3] += z
        print("z: ", z)
        print(curr_tm)
        if p5:
            jpl = ik.inv_kinematics_p5(arm, curr_tm, gangle, home_dh)
        else:
            jpl = ik.inv_kinematics(arm, curr_tm, gangle)
        self.limited[arm] = jpl[1]
        if self.limited[arm]:
            print("Desired cartesian position is out of bounds for Raven2. Will move to max pos.")
        self.new_jp = jpl[0]
        #print(self.new_jp)
        return
    
    # helper method to convert numpy array size 7 to np array size 15
    def seven2fifthteen (self, arr7):
        return np.concatenate([np.zeros(1), arr7, np.zeros(7)])
    
    # convert array size 7 to 16 
    def seven2sixteen (self, arr7):
        return np.concatenate([np.zeros(1), arr7, np.zeros(8)])
    # helper method to convert numpy array size 15 to np array size 7
    def fifthteen2seven (self, arr15):
        
        return arr15[1:8]