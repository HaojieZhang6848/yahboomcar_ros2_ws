#for patrol
#math
from math import radians, copysign, sqrt, pow
from math import pi
import numpy as np
#rclpy
import rclpy
from rclpy.node import Node
#tf
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
#msg
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
#others
import PyKDL

from yahboomcar_multi.singlePID import *

from time import sleep

print("import finish")
RAD2DEG = 180 / pi

class ListenTF(Node):
    def __init__(self,name):
        super().__init__(name)
        self.min_dist = 0.5
        self.now = self.get_clock().now().to_msg()
        self.minX = 0.0
        self.minY = 0.0
        self.source_frame = "point1"
        self.robot_frame = "robot2/base_footprint"
        self.start_status = True
        self.lin_pid = SinglePID()
        self.ang_pid = SinglePID()
        self.linPIDparam = [1.0, 0, 1.0]
        self.angPIDparam = [0.8, 0, 1.0]
        self.lin_pid.Set_pid(self.linPIDparam[0], self.linPIDparam[1], self.linPIDparam[2])
        self.ang_pid.Set_pid(self.angPIDparam[0], self.angPIDparam[1], self.angPIDparam[2])
        #create publisher
        self.pub_cmdVel = self.create_publisher(Twist,"/robot2/cmd_vel",5)
        #create subscriber
        self.sub_joy = self.create_subscription(Bool,"/JoyState",self.JoyStateCallback,1)
        #create TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        #declare param
        self.declare_parameter('Switch',True)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.declare_parameter('Navigate',True)
        self.Navigate = self.get_parameter('Navigate').get_parameter_value().bool_value
        #create timer
        #self.timer = self.create_timer(0.01,self.on_timer)
        self.index = 0
        
        
    def run(self):
        
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.Switch = self.get_parameter('Navigate').get_parameter_value().bool_value
        '''if self.navigate:
          self.listen_tf_map()  
          print("")'''
        while self.Switch == True:
            self.now = self.get_clock().now().to_msg()
            print("tf: ",self.listen_tf_xy())
            
            
    def listen_tf_xy(self):
        try:
            self.now = self.get_clock().now().to_msg()
            print("now: ",self.now)
            #now = rclpy.time.Time()
            trans= self.tf_buffer.lookup_transform(self.robot_frame,self.source_frame,self.now)
            rot= self.tf_buffer.lookup_transform(self.robot_frame,self.source_frame,self.now)
            print("trans: ",trans.transform.translation)
            print("rot: ",trans.transform.rotation)
            return trans,rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return
            
    def listen_tf_map(self):
        try:
            now = rclpy.time.Time()
            #now = self.get_clock().now().to_msg()
            trans= self.tf_buffer.lookup_transform("map",self.source_frame,now)
            rot= self.tf_buffer.lookup_transform("map",self.source_frame,now)
            print("trans: ",trans.transform.translation)
            print("rot: ",trans.transform.rotation)
            return trans,rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return
            #continue
            
            
    def listen_tf_z(self):
        try:
            #now = rclpy.time.Time()
            now = self.get_clock().now().to_msg()
            rot = self.tf_buffer.lookup_transform(self.robot_frame,self.source_frame,now)
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            angle_rot = cacl_rot.GetRPY()[2]
            return angle_rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            #continue
            
        
    def normalize_angle(self,angle):
        res = angle
        #print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
    
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
 
    
def main():
    rclpy.init()
    listen_robot1 = ListenTF("listen_robot1")
    print("create done")
    listen_robot1.run()
    rclpy.spin(listen_robot1)
    
