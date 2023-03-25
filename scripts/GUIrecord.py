#!/usr/bin/python
# coding: utf-8

import os
import tf
import sys
#load chinese language
reload(sys)  
sys.setdefaultencoding('utf8') 
import time
import copy
import rospy
import json
import threading
from Task import Task
from TaskPoint import globalTaskFinish
from TaskTransfer import TaskTransfer

from PyQt5.QtGui import QIntValidator,QDoubleValidator, QRegExpValidator
from PyQt5.Qt import (QThread,QMutex,pyqtSignal)
from PyQt5 import QtGui
from PyQt5.QtWidgets import (
    QWidget,
    QApplication,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QLineEdit,
    QTextEdit,
    QCheckBox,
    QRadioButton
)


class LocationRecorder(QWidget):
    def __init__(self):
        super(LocationRecorder, self).__init__()
        self.init_ui()
        self.robot_record_pose = None
        self.option = None
        self.tf_listener = tf.TransformListener()
        self.show()
        #hg
        self.stop_barrier = False
        self.observer_avoidance = True
        #
        self.task = Task()
        self.robot_transfer = TaskTransfer()
        #self.task_judge = self.task.run_judge
        #
        self.cond = threading.Condition()
        self.task_thread = threading.Thread(target=self.task_function, name="task_function")
        self.task_thread.setDaemon(True)
        self.task_thread.start()
        #hg_end

    def init_ui(self):
        self.layout = QVBoxLayout()
        #hg
        self.option_layout = QHBoxLayout()
        self.stair_checkbox = QCheckBox("楼梯步态1")
        self.stair_two_checkbox = QCheckBox("楼梯步态2")
        self.high_speed_checkbox = QCheckBox("高速")
        self.low_speed_checkbox = QCheckBox("低速")
        self.back_checkbox = QCheckBox("后退")
        self.autocharge_checkbox = QCheckBox("自主充电")
        # self.map_heavy_load_checkbox = QCheckBox("地图重载")
        self.visual_climbing_stairs_checkbox = QCheckBox("视觉爬楼梯")
        self.autonomous_gait_switching_checkbox = QCheckBox("自主步态切换")
        
        self.observer_avoidance_mode_radiobutton = QRadioButton("避障模式")
        self.observer_avoidance_mode_radiobutton.setChecked(True)
        self.observer_avoidance_mode_radiobutton.toggled.connect(lambda :self.obstacle_mode(self.observer_avoidance_mode_radiobutton))
        self.stop_barrier_mode_radiobutton = QRadioButton("停障模式")
        self.stop_barrier_mode_radiobutton.toggled.connect(lambda :self.obstacle_mode(self.stop_barrier_mode_radiobutton))
        #hg_end
        self.option_layout.addWidget(self.stair_checkbox)
        self.option_layout.addWidget(self.stair_two_checkbox)
        self.option_layout.addWidget(self.high_speed_checkbox)
        self.option_layout.addWidget(self.low_speed_checkbox)
        self.option_layout.addWidget(self.back_checkbox)
        self.option_layout.addWidget(self.autocharge_checkbox)
        # self.option_layout.addWidget(self.map_heavy_load_checkbox)
        self.option_layout.addWidget(self.visual_climbing_stairs_checkbox)
        self.option_layout.addWidget(self.autonomous_gait_switching_checkbox)

        self.option_layout.addWidget(self.observer_avoidance_mode_radiobutton)
        self.option_layout.addWidget(self.stop_barrier_mode_radiobutton)

        self.order_layout = QHBoxLayout()
        self.order_layout.addWidget(QLabel("路径点编号:"))
        self.order_edit = QLineEdit("")
        self.order_layout.addWidget(self.order_edit)
	#hg

        self.xy_goal_tolerance_val = rospy.get_param("/move_base/LocalPlanner/xy_goal_tolerance")
        self.yaw_goal_tolerance_val = rospy.get_param("/move_base/LocalPlanner/yaw_goal_tolerance")
        self.trans_stopped_vel_val = rospy.get_param("/move_base/LocalPlanner/trans_stopped_vel")
        self.theta_stopped_vel_val = rospy.get_param("/move_base/LocalPlanner/theta_stopped_vel")
        self.global_min_obstacle_height_val = rospy.get_param("/move_base/global_costmap/stvl_obstacle_layer/min_obstacle_height")
        self.global_max_obstacle_height_val = rospy.get_param("/move_base/global_costmap/stvl_obstacle_layer/max_obstacle_height")
        self.local_min_obstacle_height_val = rospy.get_param("/move_base/local_costmap/stvl_obstacle_layer/min_obstacle_height")
        self.local_max_obstacle_height_val = rospy.get_param("/move_base/local_costmap/stvl_obstacle_layer/max_obstacle_height")

        # 浮点校验器精度1(0.02,1.5,2)
        doubleValidator1 = QDoubleValidator(self) 
        doubleValidator1.setRange(0.02,1.5,2)
        doubleValidator1.setNotation(QDoubleValidator.StandardNotation)#标准的记号表达
        doubleValidator1.setDecimals(2)# 设置精度，小数点2位
        # 浮点校验器精度2(0.1,0.5,2)
        doubleValidator2 = QDoubleValidator(self) 
        doubleValidator2.setRange(0.1,1.5,2)
        doubleValidator2.setNotation(QDoubleValidator.StandardNotation)#标准的记号表达
        doubleValidator2.setDecimals(2)# 设置精度，小数点2位
        # 浮点校验器精度3(-1,0,2)
        doubleValidator3 = QDoubleValidator(self) 
        doubleValidator3.setRange(-1,0,2)
        doubleValidator3.setNotation(QDoubleValidator.StandardNotation)#标准的记号表达
        doubleValidator3.setDecimals(2)# 设置精度，小数点2位
        # 浮点校验器精度3(-1,0,2)
        doubleValidator4 = QDoubleValidator(self) 
        doubleValidator4.setRange(0,1,2)
        doubleValidator4.setNotation(QDoubleValidator.StandardNotation)#标准的记号表达
        doubleValidator4.setDecimals(2)# 设置精度，小数点2位

        #平面定位误差
        self.accuracy_layout = QHBoxLayout()

        self.accuracy_layout.addWidget(QLabel("平面定位误差:"))
        self.xy_goal_tolerance = QLineEdit("")  
        self.xy_goal_tolerance.setPlaceholderText("当前值："+str(self.xy_goal_tolerance_val))
        self.xy_goal_tolerance.setValidator(doubleValidator1)# 设置校验器
        self.accuracy_layout.addWidget(self.xy_goal_tolerance)
	    
        #旋转角度误差
        self.accuracy_layout.addWidget(QLabel("旋转角度误差:"))
        self.yaw_goal_tolerance = QLineEdit("")
        self.yaw_goal_tolerance.setPlaceholderText("当前值："+str(self.yaw_goal_tolerance_val))
        self.yaw_goal_tolerance.setValidator(doubleValidator1)# 设置校验器
        self.accuracy_layout.addWidget(self.yaw_goal_tolerance)

        #平动速度最大值
        self.accuracy_layout.addWidget(QLabel("平动速度设定值:"))
        self.trans_stopped_vel = QLineEdit("")
        self.trans_stopped_vel.setPlaceholderText("当前值："+str(self.trans_stopped_vel_val))
        self.trans_stopped_vel.setValidator(doubleValidator2)# 设置校验器
        self.accuracy_layout.addWidget(self.trans_stopped_vel)  
        #转动速度最大值
        self.accuracy_layout.addWidget(QLabel("转动速度设定值:"))
        self.theta_stopped_vel = QLineEdit("")
        self.theta_stopped_vel.setPlaceholderText("当前值："+str(self.theta_stopped_vel_val))
        self.theta_stopped_vel.setValidator(doubleValidator2)# 设置校验器
        self.accuracy_layout.addWidget(self.theta_stopped_vel) 
	    #全局障碍物高度最小值
        self.obstacle_layout = QHBoxLayout()
        self.obstacle_layout.addWidget(QLabel("全局障碍物高度最小值:"))
        self.global_min_obstacle_height = QLineEdit("")
        self.global_min_obstacle_height.setPlaceholderText("当前值："+str(self.global_min_obstacle_height_val))
        self.global_min_obstacle_height.setValidator(doubleValidator3)
        self.obstacle_layout.addWidget(self.global_min_obstacle_height) 
	    #全局障碍物高度最大值
        self.obstacle_layout.addWidget(QLabel("全局障碍物高度最大值):"))
        self.global_max_obstacle_height = QLineEdit("")
        self.global_max_obstacle_height.setPlaceholderText("当前值："+str(self.global_max_obstacle_height_val))
        self.global_max_obstacle_height.setValidator(doubleValidator4)
        self.obstacle_layout.addWidget(self.global_max_obstacle_height)         
	    #局部障碍物高度最小值
        self.obstacle_layout.addWidget(QLabel("局部障碍物高度最小值:"))
        self.local_min_obstacle_height = QLineEdit("")
        self.local_min_obstacle_height.setPlaceholderText("当前值："+str(self.local_min_obstacle_height_val))
        self.local_min_obstacle_height.setValidator(doubleValidator3)
        self.obstacle_layout.addWidget(self.local_min_obstacle_height) 
	    #局部障碍物高度最大值
        self.obstacle_layout.addWidget(QLabel("局部障碍物高度最大值:"))
        self.local_max_obstacle_height = QLineEdit("")
        self.local_max_obstacle_height.setPlaceholderText("当前值："+str(self.local_max_obstacle_height_val))
        self.local_max_obstacle_height.setValidator(doubleValidator4)
        self.obstacle_layout.addWidget(self.local_max_obstacle_height) 
        #hg_end

        self.text_content = QTextEdit()
        self.text_content.setEnabled(False)

        self.record_layout = QHBoxLayout()
        self.receive_button = QPushButton("获取当前位置")
        self.record_layout.addWidget(self.receive_button)

        #hg
        self.autucharge_layout = QHBoxLayout()
        self.receive_autucharge = QPushButton("记录为充电桩位置")
        self.autucharge_layout.addWidget(self.receive_autucharge)
        self.record_button = QPushButton("记录为路径点位置")
        self.autucharge_layout.addWidget(self.record_button)
        
        self.task_execution_layout = QHBoxLayout()
        self.execute_task = QPushButton("执行任务")
        self.task_execution_layout.addWidget(self.execute_task)
        self.finish_task =  QPushButton("结束任务")
        self.task_execution_layout.addWidget(self.finish_task)
        #hg_end

        self.layout.addLayout(self.option_layout)
        self.layout.addLayout(self.order_layout)
        self.layout.addLayout(self.accuracy_layout)
        self.layout.addLayout(self.obstacle_layout)
        self.layout.addWidget(self.text_content)

        self.layout.addLayout(self.record_layout)
        self.layout.addLayout(self.autucharge_layout)
        self.layout.addLayout(self.task_execution_layout)
        
        self.setLayout(self.layout)

        self.record_button.clicked.connect(self.record)
        self.receive_button.clicked.connect(self.receive)
        
        #hg
        #self.execute_task.clicked.connect(self.execute_task_function)
        #self.finish_task.clicked.connect(self.finish_task_function)
        #self.receive_autucharge.clicked.connect(self.charging_pile_record)
        #hg_end

    def record(self):
        order = self.order_edit.text()
        try:
            order = int(order)
        except:
            return
        new_record = {
            "order": order,
            "robot_pose": self.robot_record_pose,
            "option": self.option,
            "positioning_accuracy": self.positioning_accuracy,
            "obstacle_height": self.obstacle_height
        }
        data_dir = str(os.path.dirname(os.path.abspath(__file__))) + "/../data"
        os.system("mkdir -p " + data_dir)
        data_dir = data_dir + "/%d.json"
        with open(data_dir % order, "w+") as out:
            json.dump(new_record, out, indent=4)
        print ("%d Coordinates Created successfully" % order)
    #hg
    def execute_task_function(self):
        if self.task_judge == False:
            self.cond.acquire()
            self.task_judge = True
            self.cond.notify()
            self.cond.release()         
    
    def finish_task_function(self):
        if self.task_judge == True:
            self.task_judge = False
            
    def stop_function(self):
        if self.task_judge == False:
            return False    
        else:
            return True

    def task_function(self):
        while True:
            self.cond.acquire()
            self.cond.wait()
            self.task.init()
            self.task.run(self.stop_function)
            self.cond.release()
            self.robot_transfer.cancel_all_goals()
            globalTaskFinish()

            print("Task termination!!")

    def obstacle_mode(self, mode):
        if mode.text() == "避障模式" and mode.isChecked() == True:
            self.observer_avoidance = True
            self.stop_barrier = False
        if mode.text() == "停障模式" and mode.isChecked() == True:
            self.observer_avoidance = False
            self.stop_barrier = True    

    def charging_pile_record(self):
        new_charging_pile = {
            "charging_pile_pose":self.charging_pile_record_pose,
        }
        charging_pile_dir = str(os.path.dirname(os.path.abspath(__file__))) + "/../charging_pile"
        os.system("mkdir -p " + charging_pile_dir)
        charging_pile_dir = charging_pile_dir + "/Charging_pile_pose.json"
        with open(charging_pile_dir, "w+") as out:
            json.dump(new_charging_pile, out, indent=4)
        print ("Charging pile Coordinates Created successfully")

    #hg_end
    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Duration(0.0)
            )
            msg_dict = {
                "pos_x": pos[0],
                "pos_y": pos[1],
                "pos_z": pos[2],
                "ori_x": ori[0],
                "ori_y": ori[1],
                "ori_z": ori[2],
                "ori_w": ori[3],
            }
            charge_dict = {
                "pos_x": pos[0],
                "pos_y": pos[1],
                "yaw":tf.transformations.euler_from_quaternion(ori)[2]
            }
            yaw = tf.transformations.euler_from_quaternion(ori)[2]
            print "x=",pos[0]
            print "y=",pos[1]
            print "yaw=",yaw
            self.robot_record_pose = msg_dict
            self.charging_pile_record_pose = charge_dict
            return True
        except tf.Exception as e:
            print "listen to tf failed"
            return False
    def set_option(self):
        self.positioning_accuracy = {}
        #xy_goal_tolerance(0.02-1.5)
        if self.xy_goal_tolerance.text() == "":
            self.positioning_accuracy["xy_goal_tolerance"] = self.xy_goal_tolerance_val
        elif float(self.xy_goal_tolerance.text()) < 0.02:
            self.positioning_accuracy["xy_goal_tolerance"] = 0.02
        elif float(self.xy_goal_tolerance.text()) > 1.5:
            self.positioning_accuracy["xy_goal_tolerance"] = 1.5
        else:
            self.positioning_accuracy["xy_goal_tolerance"] = float(self.xy_goal_tolerance.text())
        #yaw_goal_tolerance(0.02-1.5)
        if self.yaw_goal_tolerance.text() == "":
            self.positioning_accuracy["yaw_goal_tolerance"] = self.yaw_goal_tolerance_val
        elif float(self.yaw_goal_tolerance.text()) < 0.02:
            self.positioning_accuracy["yaw_goal_tolerance"] = 0.02
        elif float(self.yaw_goal_tolerance.text()) > 1.5:
            self.positioning_accuracy["yaw_goal_tolerance"] = 1.5
        else:
            self.positioning_accuracy["yaw_goal_tolerance"] = float(self.yaw_goal_tolerance.text())
        #trans_stopped_vel(0.1-0.5)
        if self.trans_stopped_vel.text() == "":
            self.positioning_accuracy["trans_stopped_vel"] = self.trans_stopped_vel_val
        elif float(self.trans_stopped_vel.text()) < 0.1:
            self.positioning_accuracy["trans_stopped_vel"] = 0.1
        elif float(self.trans_stopped_vel.text()) > 0.5:
            self.positioning_accuracy["trans_stopped_vel"] = 0.5  
        else:
            self.positioning_accuracy["trans_stopped_vel"] = float(self.trans_stopped_vel.text())
         #theta_stopped_vel(0.1-0.5)
        if self.theta_stopped_vel.text() == "":
            self.positioning_accuracy["theta_stopped_vel"] = self.theta_stopped_vel_val
        elif float(self.theta_stopped_vel.text()) < 0.1:
            self.positioning_accuracy["theta_stopped_vel"] = 0.1
        elif float(self.theta_stopped_vel.text()) > 0.5:
            self.positioning_accuracy["theta_stopped_vel"] = 0.5
        else:
            self.positioning_accuracy["theta_stopped_vel"] = float(self.theta_stopped_vel.text())
        
        self.obstacle_height = {}
        if self.global_min_obstacle_height.text() == "":
            self.obstacle_height["global_min_obstacle_height"] = self.global_min_obstacle_height_val
        else:
            self.obstacle_height["global_min_obstacle_height"] = float(self.global_min_obstacle_height.text())
        if self.global_max_obstacle_height.text() == "":
            self.obstacle_height["global_max_obstacle_height"] = self.global_max_obstacle_height_val
        else:
            self.obstacle_height["global_max_obstacle_height"] = float(self.global_max_obstacle_height.text())
        if self.local_min_obstacle_height.text() == "":
            self.obstacle_height["local_min_obstacle_height"] = self.local_min_obstacle_height_val
        else:
            self.obstacle_height["local_min_obstacle_height"] = float(self.local_min_obstacle_height.text())
        if self.local_max_obstacle_height.text() == "":
            self.obstacle_height["local_max_obstacle_height"] = self.local_max_obstacle_height_val
        else:
            self.obstacle_height["local_max_obstacle_height"] = float(self.local_max_obstacle_height.text())
    
    def update_option(self):
        self.set_option()
        
        self.option = {}
        self.option["stair"] = self.stair_checkbox.isChecked()
        self.option["stair_two"] = self.stair_two_checkbox.isChecked()
        self.option["high_speed"] = self.high_speed_checkbox.isChecked()
        self.option["low_speed"] = self.low_speed_checkbox.isChecked()
        self.option["back"] = self.back_checkbox.isChecked()
        self.option["autocharge"] = self.autocharge_checkbox.isChecked()
        self.option["visual_climbing_stairs"] = self.visual_climbing_stairs_checkbox.isChecked()
        self.option["autonomous_gait_switching"] = self.autonomous_gait_switching_checkbox.isChecked()
        # self.option["map_heavy_load"]=self.map_heavy_load_checkbox.isChecked()
        self.option["observer_avoidance"] = self.observer_avoidance
        self.option["stop_barrier"] = self.stop_barrier
	
    def receive(self):
        while not self.listen_tf():
            rospy.sleep(1.0)
        self.update_option()
        display_msg = "Robot:\n" + json.dumps(self.robot_record_pose, indent=4) + "\n"
        display_msg += "Option:\n" + json.dumps(self.option, indent=4) + "\n"
        self.text_content.setText(display_msg)    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    rospy.init_node("pipeline_node")
    lr = LocationRecorder()
    sys.exit(app.exec_())
