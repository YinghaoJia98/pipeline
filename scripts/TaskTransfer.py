#!/usr/bin/python
# coding: utf-8
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""

from distutils.command.config import config
import rospy
import dynamic_reconfigure.client
import actionlib
from tf.transformations import *
from pipeline.msg import MoveBaseAction, MoveBaseGoal
from pipeline.srv import deep_srv
from TaskPoint import TaskPoint
from RobotCommander import RobotCommander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32


class TaskTransfer:
    def __init__(self):
        self.moveBaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.moveBaseClient.wait_for_server()
        rospy.loginfo("Action 'move_base' is up!")

        #hg
        self.xy_goal_tolerance_val = float()#rospy.get_param("/move_base/LocalPlanner/xy_goal_tolerance")
        self.yaw_goal_tolerance_val = float()#rospy.get_param("/move_base/LocalPlanner/yaw_goal_tolerance")
        self.trans_stopped_vel_val = float()#rospy.get_param("/move_base/LocalPlanner/trans_stopped_vel")
        self.theta_stopped_vel_val = float()#rospy.get_param("/move_base/LocalPlanner/theta_stopped_vel")
        self.global_min_obstacle_height_val = float()#rospy.get_param("/move_base/global_costmap/stvl_obstacle_layer/min_obstacle_height")
        self.global_max_obstacle_height_val = float()#rospy.get_param("/move_base/global_costmap/stvl_obstacle_layer/max_obstacle_height")
        self.local_min_obstacle_height_val = float()#rospy.get_param("/move_base/local_costmap/stvl_obstacle_layer/min_obstacle_height")
        self.local_max_obstacle_height_val = float()#rospy.get_param("/move_base/local_costmap/stvl_obstacle_layer/max_obstacle_height")
        #
        self.client_local_planner = dynamic_reconfigure.client.Client(
            "/move_base/LocalPlanner"
        )
        self.client_global_planner = dynamic_reconfigure.client.Client(
            "/move_base/GlobalPlanner"
        )
        self.client_move_base = dynamic_reconfigure.client.Client(
            "/move_base"
        )
        self.client_global_costmap_obstacle = dynamic_reconfigure.client.Client(
            "/move_base/global_costmap/stvl_obstacle_layer"
        )
        self.client_local_cosmap_obstacle = dynamic_reconfigure.client.Client(
            "/move_base/local_costmap/stvl_obstacle_layer"
        )
        #hg_end
         
        self.QR_is_enable=0
        rospy.Subscriber("/pose_in_apriltag", PoseStamped, self.apriltag_callback)
    
    def apriltag_callback(self,msg):
        self.QR_is_enable=1


    def plan_failed(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.ABORTED

    def is_action_succeed(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED
    
    #hg
	#
    def set_positioning_accuracy(self):
        config1 = {"theta_stopped_vel":self.theta_stopped_vel_val}
        config2 = {"xy_goal_tolerance":self.xy_goal_tolerance_val}
        config3 = {"trans_stopped_vel":self.trans_stopped_vel_val}
        config4 = {"yaw_goal_tolerance":self.yaw_goal_tolerance_val} 
        self.client_local_planner.update_configuration(config1)
        self.client_local_planner.update_configuration(config2)
        self.client_local_planner.update_configuration(config3)
        self.client_local_planner.update_configuration(config4)   
    def set_obstacle_height(self):
        #global
        config1 = {"max_obstacle_height":self.global_max_obstacle_height_val}
        config2 = {"min_obstacle_height":self.global_min_obstacle_height_val}
        #LOCAL
        config3 = {"min_obstacle_height":self.local_min_obstacle_height_val}
        config4 = {"max_obstacle_height":self.local_max_obstacle_height_val} 
        self.client_global_costmap_obstacle.update_configuration(config1)
        self.client_global_costmap_obstacle.update_configuration(config2)
        self.client_local_cosmap_obstacle.update_configuration(config3)
        self.client_local_cosmap_obstacle.update_configuration(config4)
    #
    def set_stairt_costmap_disable(self):
        config = {"enabled": False}
        self.client_local_cosmap_obstacle.update_configuration(config)
        print("close local_costmap")
        self.client_global_costmap_obstacle.update_configuration(config)
        print("close global_costmap")

    def set_stairt_costmap_enable(self):
        config = {"enabled": True}
        self.client_local_cosmap_obstacle.update_configuration(config)
        print("open local_costmap")
        self.client_global_costmap_obstacle.update_configuration(config)
        print("open global_costmap")

    def stop_barrier_mode(self):
        config = {"planner_frequency": 0, "replanning_enabled": False}
        self.client_move_base.update_configuration(config)
    
    def observer_avoidance_mode(self):
        config = {"planner_frequency": 5, "replanning_enabled": True}
        self.client_move_base.update_configuration(config)

    def set_forward_mode(self):
        config = {"orientation_mode": 1}
        self.client_global_planner.update_configuration(config)
        config = {"weight_kinematics_forward_drive": 10}
        self.client_local_planner.update_configuration(config)
    #
    def set_backward_mode(self):
        config = {"orientation_mode": 4}
        self.client_global_planner.update_configuration(config)
        config = {"weight_kinematics_forward_drive": 0.1}
        self.client_local_planner.update_configuration(config)
    #
    def set_low_speed(self):
        config = {
            "max_vel_x": 0.3,
            "max_vel_x_backwards": 0.1,
            "max_vel_y": 0.05,
            "max_vel_theta": 0.5,
            "acc_lim_x": 1.0,
            "acc_lim_y": 0.05,
            "acc_lim_theta": 0.3,
        }
        self.client_local_planner.update_configuration(config)

    def set_normal_speed(self):
        config = {
            "max_vel_x": 0.5,
            "max_vel_x_backwards": 0.2,
            "max_vel_y": 0.1,
            "max_vel_theta": 0.5,
            "acc_lim_x": 1.0,
            "acc_lim_y": 0.05,
            "acc_lim_theta": 0.3,
        }
        self.client_local_planner.update_configuration(config)

    def set_high_speed(self):
        config = {
            "max_vel_x": 1.0,
            "max_vel_x_backwards": 0.3,
            "max_vel_y": 0.04,
            "max_vel_theta": 0.5,
            "acc_lim_x": 1.0,
            "acc_lim_y": 0.05,
            "acc_lim_theta": 0.3,
        }
        self.client_local_planner.update_configuration(config)
    #hg_end

    def task_transfer(self, src_point, des_point):
        """
        Main Decision Function
        """
        # with RobotCommander() as robot_commander:
        #     robot_commander.sendCordinate(
        #         command_code=51,
        #         x=src_point.getPosX(),
        #         y=src_point.getPosY(),
        #         yaw=src_point.getYaw(),
        #     )
        #     print ("-----------task_transfer-----------")
        des_point.setPreTaskPoint(src_point)

        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.pose.position.x = des_point.getPosX()
        goal_msg.target_pose.pose.position.y = des_point.getPosY()
        goal_msg.target_pose.pose.position.z = 0
        my_q = quaternion_from_euler(0, 0, des_point.getYaw())
        goal_msg.target_pose.pose.orientation.x = my_q[0]
        goal_msg.target_pose.pose.orientation.y = my_q[1]
        goal_msg.target_pose.pose.orientation.z = my_q[2]
        goal_msg.target_pose.pose.orientation.w = my_q[3]

        print des_point.record["option"]

        # not_done = True
        # while not_done and not rospy.is_shutdown():
        #     """
        #     Do something REPEATEDLY
        #     """
        #     self.moveBaseClient.send_goal(goal_msg)
        #     rospy.logwarn(
        #         "Transfer from [%s] to [%s]" % (src_point.name, des_point.name)
        #     )
        #     rospy.sleep(5)

        #     done = self.moveBaseClient.wait_for_result(timeout=rospy.Duration(5.0))
        #     not_done = (not done) or (
        #         self.moveBaseClient.get_state() != actionlib.GoalStatus.SUCCEEDED
        #     )

        #hg
        not_done = True
        if not rospy.is_shutdown():
            """
            Do something REPEATEDLY
            """
            self.moveBaseClient.send_goal(goal_msg)
            rospy.logwarn(
                "Transfer from [%s] to [%s]" % (src_point.name, des_point.name)
            )
            while not_done == True:
                if self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED:
                	not_done = False
        else:
            print("moveBaseClient_send_goal failed!!!")
        #hg_end
        """
        Do something to finish the action, only ONCE
        """ 
        #hg
        #精度处理、障碍物处理
        self.xy_goal_tolerance_val = des_point.is_positioning_accuracy().get("xy_goal_tolerance")
        self.yaw_goal_tolerance_val = des_point.is_positioning_accuracy().get("yaw_goal_tolerance")
        self.trans_stopped_vel_val = des_point.is_positioning_accuracy().get("trans_stopped_vel")
        self.theta_stopped_vel_val = des_point.is_positioning_accuracy().get("theta_stopped_vel")
        self.global_min_obstacle_height_val = des_point.is_obstacle_height().get("global_min_obstacle_height")
        self.global_max_obstacle_height_val = des_point.is_obstacle_height().get("global_max_obstacle_height")
        self.local_min_obstacle_height_val = des_point.is_obstacle_height().get("local_min_obstacle_height")
        self.local_max_obstacle_height_val = des_point.is_obstacle_height().get("local_max_obstacle_height")
        if not self.plan_failed():
            self.set_positioning_accuracy()
            print ("positioning_accuracy SETUP SUCCESS.")
            self.set_obstacle_height()
            print ("obstacle_height SETUP SUCCESS.")
        #
        if not self.plan_failed():
            if des_point.is_back():
                print ("BACKWARD...")
                self.set_backward_mode()
                rospy.sleep(0.2)
            else:
                print ("FORWARD...")
                self.set_forward_mode()
                rospy.sleep(0.2)
                
        if not self.plan_failed():
            if des_point.is_high_speed() and not des_point.is_low_speed():
                print ("START TO HIGH SPEED...")
                self.set_high_speed()
                rospy.sleep(0.2)
            elif not des_point.is_high_speed() and des_point.is_low_speed():
                print ("START TO LOW SPEED...")
                self.set_low_speed()
                rospy.sleep(0.2)
            else: 
                print ("START TO NORMAL SPEED...")
                self.set_normal_speed()
                rospy.sleep(0.2)
        

        if not self.plan_failed():
                # with RobotCommander() as robot_commander:
                #self. basic_command_client(4, 0,0,0,0)
                if des_point.is_stair() == True and des_point.is_stair_two() == False: 
                    print ("START TO STAIRS ONE...")
                    # robot_commander.stair_trait()
                    self.basic_command_client(5, 1,0,0,0)
                    self.set_stairt_costmap_disable()
                    rospy.sleep(0.2)
                elif des_point.is_stair() == False and des_point.is_stair_two() == True:
                    print ("START TO STAIRS TWO...")
                    # robot_commander.stair_trait_two()
                    self. basic_command_client(5, 2,0,0,0)
                    self.set_stairt_costmap_disable()
                    rospy.sleep(0.2)
                else:
                    print ("WALK...") 
                    # robot_commander.finish_trait_two()
                    self. basic_command_client(5, 0,0,0,0)
                    # self. basic_command_client(4, 1)
                    self.set_stairt_costmap_enable()
                    rospy.sleep(0.2)  
       

        if not self.plan_failed():
            if des_point.is_observer_avoidance() == True and des_point.is_stop_barrier() == False:
                print ("OBSERVER AVOIDANCE MODE...")
                self.observer_avoidance_mode()
                rospy.sleep(0.2)
            elif des_point.is_observer_avoidance() == False and des_point.is_stop_barrier() ==  True:
                print ("STOP BARRIER MODE...")
                self.stop_barrier_mode()
                rospy.sleep(0.2)
            else:
                print ("MODE ERROR...")   
        #
        if not self.plan_failed():
            if des_point.is_autocharge():
                    self.QR_is_enable=0
                    print ("START TO UP AUTOCHARGE...")
                    self.basic_command_client(3, 0,0,0,0)
                    pub=rospy.Publisher('/autocharge_manager',Int32,queue_size=10)
                    msg=Int32()
                    msg.data=1
                    pub.publish(msg)
                    rospy.sleep(0.5)
                    pub.publish(msg)
                    rospy.sleep(0.5)
                    pub.publish(msg)
                    print("pub on")
                    rospy.sleep(5)
                    if self.QR_is_enable==1:
                        self.basic_command_client(3,1,0,0,0)
                        self. basic_command_client(7,0,0,0,0)
                        rospy.sleep(60)
                        self. basic_command_client(7,1,0,0,0)
                        msg.data=0
                        pub.publish(msg)
                        rospy.sleep(0.5)
                        pub.publish(msg)
                        print("pub off")
                    elif self.QR_is_enable==0:
                        self.basic_command_client(1, 0,0,0,0)

                     
        #hg_end
    #hg
    def cancel_all_goals(self):
        self.moveBaseClient.cancel_all_goals()
        rospy.loginfo("Client cancel all goals")
    #hg_end

    def basic_command_client(self,commandCode, functionCode,x,y,yaw):
        rospy.wait_for_service('basic_command')
        try:
            basic_command = rospy.ServiceProxy('basic_command',deep_srv)
            resp = basic_command (commandCode, functionCode,x,y,yaw)
            if resp.c == -1:
                basic_command (6, 0,0,0,0)
        except rospy.ServiceException as e:
            print("Service call failed: %s " %e)
