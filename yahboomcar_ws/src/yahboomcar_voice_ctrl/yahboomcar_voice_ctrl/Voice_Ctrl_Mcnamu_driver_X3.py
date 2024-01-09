#!/usr/bin/env python
# encoding: utf-8

#public lib
import sys
import math
import random
import threading
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster
from Speech_Lib import Speech
import time

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock

#from dynamic_reconfigure.server import Server
car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}
spe = Speech()
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		self.car = Rosmaster()
		self.car.set_car_type(1)
		#get parameter
		self.declare_parameter('car_type', 'X3')
		self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
		print (self.car_type)
		self.declare_parameter('imu_link', 'imu_link')
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('angular_limit', 1.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)

		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

		#create publisher
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.volPublisher = self.create_publisher(Float32,"voltage",100)
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"/imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",100)

		#create timer
		self.timer = self.create_timer(0.1, self.pub_data)

		#create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		self.car.create_receive_threading()
	#callback function
	def cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y
		angular = msg.angular.z     # wait for change
		self.car.set_car_motion(vx*1.8, vy, angular)
        #rospy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)
	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)
	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	#pub data
	def pub_data(self):
		speech_r = spe.speech_read()
		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		if len(self.Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
				
		edition.data = self.car.get_version()
		battery.data = self.car.get_battery_voltage()
		ax, ay, az = self.car.get_accelerometer_data()
		gx, gy, gz = self.car.get_gyroscope_data()
		mx, my, mz = self.car.get_magnetometer_data()
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()
		
		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		imu.angular_velocity.z = gz*1.0

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0
		
		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx    #velocity in axis 
		twist.linear.y = vy
		twist.angular.z = angular    #this is invalued
		self.velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rospy.loginfo("battery: {}".format(battery))
		# rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
		self.imuPublisher.publish(imu)
		self.magPublisher.publish(mag)
		self.volPublisher.publish(battery)
		self.EdiPublisher.publish(edition)
		#speech
        #print(speech_r)
		if speech_r == 2 or speech_r == 0 :
			vx = 0.0
			vy = 0.0
			angular = 0
			self.car.set_car_motion(vx, vy, angular)
			spe.void_write(speech_r)
# 前进
		elif speech_r == 4 :
			print("Go ahead!")       
			vx = 0.5
			vy = 0.0
			angular = 0
			self.car.set_car_motion(vx, vy, angular)
			spe.void_write(speech_r)
			time.sleep(5)
			vx = 0.0
			vy = 0.0
			angular = 0
			self.car.set_car_motion(vx, vy, angular)
# 后退
		elif speech_r == 5 :
			print("Back off!")
			vx = -0.5
			vy = 0.0
			angular = 0
			self.car.set_car_motion(vx, vy, angular)
			spe.void_write(speech_r)
			time.sleep(5)
			vx = 0.0
			vy = 0.0
			angular = 0
			self.car.set_car_motion(vx, vy, angular)
# 左转
		elif speech_r == 6 :
			print("Turn left")
			vx = 0.5
			vy = 0.0
			angular = 0.2
			car.set_car_motion(vx, vy, angular)
			spe.void_write(speech_r)
			time.sleep(5)
			vx = 0.0
			vy = 0.0
			angular = 0
			self.car.set_car_motion(vx, vy, angular)
# 右转    
		elif speech_r == 7 :
			print("Turn right")
			vx = 0.5
			vy = 0.0
			angular = -0.2
			self.car.set_car_motion(vx, vy, angular) 
			spe.void_write(speech_r)
			time.sleep(5)
			vx = 0.0
			vy = 0.0
			angular = 0
			self.car.set_car_motion(vx, vy, angular)

		elif speech_r == 11 :
			print("Red light")
			self.car.set_colorful_lamps(0xFF,255,0,0)
			spe.void_write(speech_r) 

		elif speech_r == 10 :
			print("Close light")
			self.car.set_colorful_effect(0, 6, parm=1)
			spe.void_write(speech_r)
# 亮绿灯
		elif speech_r == 12 :
			print("Green light")
			self.car.set_colorful_lamps(0xFF,0,255,0)
			spe.void_write(speech_r)
# 亮蓝灯
		elif speech_r == 13 :
			print("Blue light")
			self.car.set_colorful_lamps(0xFF,0,0,255)
			spe.void_write(speech_r)
# 亮黄灯
		elif speech_r == 14 :
			print("Yellow light")
			self.car.set_colorful_lamps(0xFF,255,255,0)
			spe.void_write(speech_r)
# 打开流水灯
		elif speech_r == 15 :
			print("Water lamps")
			self.car.set_colorful_effect(1, 6, parm=1)
			spe.void_write(speech_r)
# 打开渐变灯
		elif speech_r == 16 :
			print("Gradient light")
			self.car.set_colorful_effect(4, 6, parm=1)
			spe.void_write(speech_r)
# 打开呼吸灯
		elif speech_r == 17 :
			print("Breathing light")
			self.car.set_colorful_effect(3, 6, parm=1)
			spe.void_write(speech_r)
# 显示电量
		elif speech_r == 18 :
			print("Display electricity")
			self.car.set_colorful_effect(6, 6, parm=1)
			spe.void_write(speech_r)
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)

		
		
