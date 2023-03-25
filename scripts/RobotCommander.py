#!/usr/bin/python
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""

import socket
import struct
import rospy
import threading
import os 
import roslaunch
from std_msgs.msg import Int32

class RobotCommander:
    """
    RobotCommander is resposible for command the robot to stop, tread or play.
    """

    _command_code = {
        "STAND_UP_DOWN": 0x31010202,
        "START_FORCE_MODE": 0x3101020a,
        "MOTION_START_STOP": 0x31010201, 
        "HEART_BEAT": 21,
    }

    def __init__(self, local_port=20001, ctrl_ip="192.168.1.120", ctrl_port=43893):
        self.local_port = local_port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.ctrl_addr = (ctrl_ip, ctrl_port)

    def __enter__(self):
        # print "on enter"
        # self.server.bind(("0.0.0.0", self.local_port))
        self._keep_alive = True
        self.comm_lock = threading.Lock()
        self.keep_alive_thread = threading.Thread(
            target=self.keep_alive, name="keep_alive"
        )
        self.keep_alive_thread.setDaemon(True)
        self.keep_alive_thread.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # print "on exit"
        self.server = None
        self._keep_alive = False
        self.keep_alive_thread.join()

    def keep_alive(self):
        while not rospy.is_shutdown() and self._keep_alive:
            if self.server:
                self.sendSimpleCommand("HEART_BEAT", verbose=False)
                rospy.sleep(0.5)

    def sendSimple(self, command_code, command_value=0, command_type=0):
        data = struct.pack("<3i", command_code, command_value, command_type)
        (1, 0, 0)
        self.comm_lock.acquire()
        if self.server:
            self.server.sendto(data, self.ctrl_addr)
        self.comm_lock.release()

    def sendCordinate(self, command_code=51, x=0.0, y=0.0, yaw=0.0):
        data = struct.pack("<3i3d", command_code, 24, 1, x, y, yaw)
        # if command_code == 51:
        #     print "111111"
        self.comm_lock.acquire()
        self.server.sendto(data, self.ctrl_addr)
        self.comm_lock.release()

    def sendSimpleCommand(self, command_name, verbose=True):
        if verbose:
            rospy.loginfo(command_name)
        self.sendSimple(self._command_code[command_name])

    def stand_down_up(self):
        self.sendSimpleCommand("STAND_UP_DOWN")

    def start_force_mode(self):
        self.sendSimpleCommand("START_FORCE_MODE")

    def motion_start_stop(self):
        self.sendSimpleCommand("MOTION_START_STOP")

    def yaw_adjust(self, adjust_rad):
        self.sendSimple(33, int(adjust_rad * 1000))


    #hg
    def stair_trait(self):
        self.sendSimple(0x31010401)
        rospy.sleep(0.1)
        
    def finish_stair_trait(self):
        self.sendSimple(0x31010300)
        rospy.sleep(0.1)

    def stair_trait_two(self):
        self.sendSimple(0x31010405)
        rospy.sleep(0.1)
        
    def finish_trait_two(self):
        self.sendSimple(0x31010300)
        rospy.sleep(0.1)

    #
    def autocharge_trait(self):
        self.sendSimple(50,4)    
        rospy.sleep(120)
        self.sendSimple(50,8)

      
    



    #hg_end

    

# keep alive
robot_commander = RobotCommander(local_port=20002)
robot_commander.__enter__()
