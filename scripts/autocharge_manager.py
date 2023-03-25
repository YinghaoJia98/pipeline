import os
import rospy
import roslaunch
from std_msgs.msg import Int32


class camera_manager:
        def __init__(self):
            self.switch=2
            self.is_open=False

            rospy.init_node("autocharge_manager_node", anonymous=False)
            self.rate=rospy.Rate(30)

            rospy.Subscriber("/autocharge_manager", Int32, self.msg_callback) 
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch_ = roslaunch.parent.ROSLaunchParent(
            self.uuid, ["/home/ysc/realsense-driver/src/realsense-ros/realsense2_camera/launch/rs_camera_color.launch",
                        "/home/ysc/deeprobotics_perception/common_fuctions_ws/src/Apriltags2_VO/apriltags2_ros/launch/continuous_detection.launch"])

            node_list=["/Image2RTSPNodelet",
                       "/camera_front_down/realsense2_camera",
                       "/camera_front_down/realsense2_camera_manager",
                       "/camera_front_up/realsense2_camera",
                       "/camera_front_up/realsense2_camera_manager",
                       "/camera_right/realsense2_camera",
                       "/camera_right/realsense2_camera_manager",
                       "/continuous_detection.launch"
                       ] 
            for node in node_list:
                cmd="rosnode kill"+" "+node
                os.system(cmd)

            self.run()
      
      
  
        def msg_callback(self,msg):
            self.switch=msg.data


        def run(self):
            while not rospy.is_shutdown():
                if self.switch==1:      
                        if self.is_open==False:                 
                           self.launch_.start()
                           self.is_open=True
                           print("on")
                        else:
                            print("pro is already opened!")
                        self.switch=2
                elif self.switch==0:
                        self.launch_.shutdown()
                        self.is_open=False
                        self.switch=2
                        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                        roslaunch.configure_logging(self.uuid)
                        self.launch_ = roslaunch.parent.ROSLaunchParent(
                        self.uuid, ["/home/ysc/realsense-driver/src/realsense-ros/realsense2_camera/launch/rs_camera_color.launch",
                                    "/home/ysc/deeprobotics_perception/common_fuctions_ws/src/Apriltags2_VO/apriltags2_ros/launch/continuous_detection.launch"])
                        print("off")

                
                
                self.rate.sleep()

RUN=camera_manager()
    

  


