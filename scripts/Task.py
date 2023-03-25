#!/usr/bin/python
# """
# Copyright (c) Deep Robotics Inc. - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
# Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
# """

import os
import re
import json
import rospy
from TaskPoint import TaskPoint, globalTaskPrepare
from TaskTransfer import TaskTransfer
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
from RobotCommander import RobotCommander
import threading
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8,Int32
from tf import transformations
from pipeline.msg import ScanMatchingStatus
import socket
import struct


class Task:
    def __init__(self):
        self.taskPoints = []
        self.currentIndex = 0
        self.robot_transfer = None
        self.src_index = None
        self.des_index = None
        self.ntask = 0
        self.posx=0
        self.posy=0
        self.posz=0
        self.yaw =0
        #self.tf_listener = tf.TransformListener()

        #self.count_list=[]
        self.count=0



     #   self.robot_state=0
     #   rospy.Subscriber("/status", ScanMatchingStatus, self.location_status_callback)
     #   rospy.Subscriber("/robot_basic_state", Int32, self.robot_status_callback)
      

   

    # def location_status_callback(self,msg):
    #     # print(msg.inlier_fraction)
    #     #  self.count_list.append(msg.inlier_fraction)
    #     #  if self.count_list.__len__()==10:
    #     #     count=0
    #     #     for i in self.count_list:
    #     #         if i >0.7:
    #     #             break
    #     #         else:
    #     #             count=count+1
            
    #     #     if count==10:
    #     #         print("location loss!!!")
    #     #         if self.robot_state !=6:
    #     #             with RobotCommander() as robot_commander:
    #     #                 robot_commander.sendSimple(0x31010c0e)
    #     #                 print("emergency stop!!!")
            
    #     #     del self.count_list[:]
    #     if msg.matching_error>0.9:
    #         self.count=self.count+1
    #         if self.count==10:
    #             print("location loss!!!")
    #             if self.robot_state !=6:
    #                  with RobotCommander() as robot_commander:
    #                      robot_commander.sendSimple(0x31010c0e)
    #                      print("emergency stop!!!")
    #             self.count=0
    #     else:
    #         self.count=0



             

    #def robot_status_callback(self,msg):
    #    self.robot_state=msg.data          

    

    def init(self):
        # stand up, read to go
        globalTaskPrepare()
        # from task_point1 to task_point2
        self.robot_transfer = TaskTransfer()
        self.loadTaskpoints()
        # only using TaskInit() to get nearest task_point
        task_init = TaskInit()
        nearest_index, initial_point = task_init.getBestTaskInd(self.taskPoints)
        self.robot_transfer.task_transfer(initial_point, self.taskPoints[nearest_index])
        # total number of the task_points
        self.ntask = self.taskPoints.__len__()
        self.src_index = nearest_index
        self.des_index = (nearest_index + 1) % self.ntask

    def loadTaskpoints(self):
        folder = str(os.path.dirname(os.path.abspath(__file__))) + "/../data"
        task_json = None
        if os.path.exists(folder):
            task_json = os.listdir(folder)
        if not task_json:
            raise Exception("No valid task point to tranverse!")

        task_list = []
        for i, file_name in enumerate(task_json):
            with open(folder + "/" + file_name, "r") as json_fp:
                waypoint_record = json.load(json_fp)
                task_list.append(waypoint_record)
        task_list = sorted(task_list, key=lambda s: s["order"])
        for waypoint_record in task_list:
            self.taskPoints.append(TaskPoint(waypoint_record))

    def run(self):
        while  not rospy.is_shutdown():
                self.robot_transfer.task_transfer(
                    self.taskPoints[self.src_index], self.taskPoints[self.des_index]
                )
                self.src_index = self.des_index
                self.des_index = (self.des_index + 1) % self.ntask
    
  
class TaskInit:
    def __init__(self):
        self.initialPose = None
        #self.tf_listener = tf.TransformListener()
        self.pos=[0,0,0]
        self.ori=[0,0,0,0]
        rospy.Subscriber("/odom", Odometry, self.odom_msg_callback)
      

    def odom_msg_callback(self,msg):
            self.pos=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
            self.ori=[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]


    def sub_odom(self):
        try:
            print ("pos: ", self.pos)
            print ("ori: ", self.ori)
            msg_list = [self.pos[0], self.pos[1], self.pos[2], self.ori[0], self.ori[1], self.ori[2], self.ori[3]]
            self.initialPose = msg_list
            return True
        except :
            print("no odom message")
            return False

    def refreshInitialPose(self):
        self.initialPose = None
        RATE = 50
        while not self.initialPose:
            self.sub_odom()
            rospy.sleep(1.0 / RATE)

    def getBestTaskInd(self, task_points):
        self.refreshInitialPose()
        fake_task = TaskPoint()
        fake_task.setRobotPose(self.initialPose)
        dist_list = [fake_task.calDistance(task_point) for task_point in task_points]
        return np.argmin(np.array(dist_list)), fake_task

if __name__ =="__main__":
    rospy.init_node("task_node")
    task=Task()
    task.init()
    task.run()