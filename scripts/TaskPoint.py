#!/usr/bin/python
# coding: utf-8

import time
from tf import transformations
from RobotCommander import RobotCommander
import rospy
from pipeline.srv import deep_srv
import tf
import copy


class TaskPoint:
    def __init__(self, record=None):
        #hg
        #对0号点的定位精度、障碍物高度参数进行默认初始化
        self.xy_goal_tolerance_val = rospy.get_param("/move_base/LocalPlanner/xy_goal_tolerance")
        self.yaw_goal_tolerance_val = rospy.get_param("/move_base/LocalPlanner/yaw_goal_tolerance")
        self.trans_stopped_vel_val = rospy.get_param("/move_base/LocalPlanner/trans_stopped_vel")
        self.theta_stopped_vel_val = rospy.get_param("/move_base/LocalPlanner/theta_stopped_vel")
        self.global_min_obstacle_height_val = rospy.get_param("/move_base/global_costmap/stvl_obstacle_layer/min_obstacle_height")
        self.global_max_obstacle_height_val = rospy.get_param("/move_base/global_costmap/stvl_obstacle_layer/max_obstacle_height")
        self.local_min_obstacle_height_val = rospy.get_param("/move_base/local_costmap/stvl_obstacle_layer/min_obstacle_height")
        self.local_max_obstacle_height_val = rospy.get_param("/move_base/local_costmap/stvl_obstacle_layer/max_obstacle_height")
        #hg_end
        if not record:
            record = {
                "order": 0,
                "robot_pose": {
                    "pos_x": 0.0,
                    "pos_y": 0.0,
                    "pos_z": 0.0,
                    "ori_x": 0.0,
                    "ori_y": 0.0,
                    "ori_z": 0.0,
                    "ori_w": 1.0,
                },
                "option": {
                    "stair": False,
                    "stair_two":False,
                    "high_speed": False,
                    "low_speed": False,
                    "back": False,
                    "autocharge": False,
                    "visual_climbing_stairs": False,
                    "autonomous_gait_switching": False,
                    "observer_avoidance": True,
                    "stop_barrier": False
                },
		"positioning_accuracy": {
        	"theta_stopped_vel": self.theta_stopped_vel_val, 
        	"xy_goal_tolerance": self.xy_goal_tolerance_val, 
        	"trans_stopped_vel": self.trans_stopped_vel_val, 
        	"yaw_goal_tolerance": self.yaw_goal_tolerance_val
    		}, 
    		"obstacle_height": {
        	"global_max_obstacle_height": self.global_min_obstacle_height_val, 
        	"local_min_obstacle_height": self.local_min_obstacle_height_val, 
        	"global_min_obstacle_height": self.global_max_obstacle_height_val, 
        	"local_max_obstacle_height": self.local_max_obstacle_height_val
    		}, 
            }
        self.pre_task_point = None
        self.record = copy.deepcopy(record)
        self.tf_listener = tf.TransformListener()
        self.update()

    def setPreTaskPoint(self, src_point):
        self.pre_task_point = src_point

    def getPreTaskPoint(self):
        return self.pre_task_point
        # return self.pre_task_point.record["order"]

    def order_equal_to(self, num):
        return self.record["order"] == num

    #hg
    def is_stair(self):
        return self.record["option"]["stair"]

    def is_stair_two(self):
        return self.record["option"]["stair_two"]

    def is_high_speed(self):
        return self.record["option"]["high_speed"]

    def is_low_speed(self):
        return self.record["option"]["low_speed"]

    def is_back(self):
        return self.record["option"]["back"]

    def is_autocharge(self):
        return self.record["option"]["autocharge"]
    
    def is_observer_avoidance(self):
        return self.record["option"]["observer_avoidance"]

    def is_stop_barrier(self):
        return self.record["option"]["stop_barrier"]

    def is_visual_climbing_stairs(self):
        return self.record["option"]["visual_climbing_stairs"]

    def is_autonomous_gait_switching(self):
        return self.record["option"]["autonomous_gait_switching"]

    def is_positioning_accuracy(self):
        return self.record["positioning_accuracy"]
        
    def is_obstacle_height(self):
        return self.record["obstacle_height"]
    #hg_end

    def update(self):
        self.robot_pose = self.record["robot_pose"]
        pose = []
        pose.append(self.robot_pose["pos_x"])
        pose.append(self.robot_pose["pos_y"])
        pose.append(self.robot_pose["pos_z"])
        pose.append(self.robot_pose["ori_x"])
        pose.append(self.robot_pose["ori_y"])
        pose.append(self.robot_pose["ori_z"])
        pose.append(self.robot_pose["ori_w"])
        self.posX = pose[0]
        self.posY = pose[1]
        #hg
        self.posZ = pose[2]
        #hg_end
        self.yaw = transformations.euler_from_quaternion(pose[3:])[2]
        self.name = "waypoint_" + str(self.record["order"])

    def setRobotPose(self, robot_pose):
        self.record["robot_pose"]["pos_x"] = robot_pose[0]
        self.record["robot_pose"]["pos_y"] = robot_pose[1]
        self.record["robot_pose"]["pos_z"] = robot_pose[2]
        self.record["robot_pose"]["ori_x"] = robot_pose[3]
        self.record["robot_pose"]["ori_y"] = robot_pose[4]
        self.record["robot_pose"]["ori_z"] = robot_pose[5]
        self.record["robot_pose"]["ori_w"] = robot_pose[6]
        self.update()

    def getPosX(self):
        return self.posX

    def getPosY(self):
        return self.posY

    #hg
    def getPosZ(self):
        return self.posZ
    #hg_end

    def getYaw(self):
        return self.yaw

    #hg 
    def calDistance(self, other):
        return (
            (self.getPosX() - other.getPosX()) ** 2
            + (self.getPosY() - other.getPosY()) ** 2
            + (self.getPosZ() - other.getPosZ()) ** 2
        ) ** 0.5
    #hg_end


def globalTaskPrepare():
     # with RobotCommander() as robot_commander:
        # robot_commander.stand_down_up()

        basic_command_client(1, 1,0,0,0)
        # rospy.sleep(5.0)
        # robot_commander.start_force_mode()
        basic_command_client(2, 0,0,0,0)
        # rospy.sleep(2.0)
        # robot_commander.motion_start_stop()
        basic_command_client(3, 1,0,0,0)
        # rospy.sleep(2.0)
        #basic_command_client(6, 0,0,0,0)
        # rospy.sleep(2.0)


def globalTaskFinish():
   # with RobotCommander() as robot_commander:
        # robot_commander.motion_start_stop()
        basic_command_client(3, 1,0,0,0)
        # rospy.sleep(2.0)
        # robot_commander.stand_down_up()
        basic_command_client(1, 1,0,0,0)
        # rospy.sleep(5.0)

def basic_command_client(commandCode, functionCode,x,y,yaw):
    rospy.wait_for_service('basic_command')
    try:
        basic_command = rospy.ServiceProxy('basic_command',deep_srv)
        resp = basic_command (commandCode, functionCode,x,y,yaw)
        if resp.c == -1:
            basic_command (6, 0,0,0,0)
    except rospy.ServiceException as e:
        print("Service call failed: %s " %e)