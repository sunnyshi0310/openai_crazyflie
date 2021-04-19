#!/usr/bin/env python

import numpy as np
import rospy
import time
from mav_msgs.msg import DroneState # message
from nav_msgs.msg import Odometry

class cf_consensus():
	def __init__(self):

		self.cf0 = DroneState()
		self.cf1 = DroneState()
		self.cf2 = DroneState()
		self.cf3 = DroneState()

		rospy.init_node('cf_trajectory')
		self.pub0 = rospy.Publisher("/crazyflie2_1/drone_state", DroneState, queue_size =10)
		self.pub1 = rospy.Publisher("/crazyflie2_2/drone_state", DroneState, queue_size =10)
		self.pub2 = rospy.Publisher("/crazyflie2_3/drone_state", DroneState, queue_size =10)
		self.pub3 = rospy.Publisher("/crazyflie2_4/drone_state", DroneState, queue_size =10)

		# Subscriber
		rospy.Subscriber("/crazyflie2_1/odometry",Odometry,self.callback0)
		rospy.Subscriber("/crazyflie2_2/odometry",Odometry,self.callback1)
		rospy.Subscriber("/crazyflie2_3/odometry",Odometry,self.callback2)
		rospy.Subscriber("/crazyflie2_4/odometry",Odometry,self.callback3)

		# Initialization
		# CRAZYFLIE_0
		self.cf0.position.x = 0.0
		self.cf0.position.y = -1.0
		self.cf0.position.z = 1.0

		self.cf0.linear_velocity.x = 0.0
		self.cf0.linear_velocity.y = 0.0
		self.cf0.linear_velocity.z = 0.0

		self.cf0.linear_acceleration.x = 0.0
		self.cf0.linear_acceleration.y = 0.0
		self.cf0.linear_acceleration.z = 0.0

		self.cf0.orientation.x = 0.0
		self.cf0.orientation.y = 0.0
		self.cf0.orientation.z = 0.0
		self.cf0.orientation.w = 1.0

		self.cf0.angular_velocity.x = 0.0
		self.cf0.angular_velocity.y = 0.0
		self.cf0.angular_velocity.z = 0.0

		self.cf0.angular_acceleration.x = 0.0
		self.cf0.angular_acceleration.y = 0.0
		self.cf0.angular_acceleration.z = 0.0

		# CRAZYFLIE_1
		self.cf1.position.x = 0.0
		self.cf1.position.y = -0.5
		self.cf1.position.z = 1.0

		self.cf1.linear_velocity.x = 0.0
		self.cf1.linear_velocity.y = 0.0
		self.cf1.linear_velocity.z = 0.0

		self.cf1.linear_acceleration.x = 0.0
		self.cf1.linear_acceleration.y = 0.0
		self.cf1.linear_acceleration.z = 0.0

		self.cf1.orientation.x = 0.0
		self.cf1.orientation.y = 0.0
		self.cf1.orientation.z = 0.0
		self.cf1.orientation.w = 1.0

		self.cf1.angular_velocity.x = 0.0
		self.cf1.angular_velocity.y = 0.0
		self.cf1.angular_velocity.z = 0.0

		self.cf1.angular_acceleration.x = 0.0
		self.cf1.angular_acceleration.y = 0.0
		self.cf1.angular_acceleration.z = 0.0

		# CRAZYFLIE_2
		self.cf2.position.x = 0.0
		self.cf2.position.y = 0.0
		self.cf2.position.z = 1.0

		self.cf2.linear_velocity.x = 0.0
		self.cf2.linear_velocity.y = 0.0
		self.cf2.linear_velocity.z = 0.0

		self.cf2.linear_acceleration.x = 0.0
		self.cf2.linear_acceleration.y = 0.0
		self.cf2.linear_acceleration.z = 0.0

		self.cf2.orientation.x = 0.0
		self.cf2.orientation.y = 0.0
		self.cf2.orientation.z = 0.0
		self.cf2.orientation.w = 1.0

		self.cf2.angular_velocity.x = 0.0
		self.cf2.angular_velocity.y = 0.0
		self.cf2.angular_velocity.z = 0.0

		self.cf2.angular_acceleration.x = 0.0
		self.cf2.angular_acceleration.y = 0.0
		self.cf2.angular_acceleration.z = 0.0

		# CRAZYFLIE_3
		self.cf3.position.x = 0.0
		self.cf3.position.y = 0.5
		self.cf3.position.z = 1.0

		self.cf3.linear_velocity.x = 0.0
		self.cf3.linear_velocity.y = 0.0
		self.cf3.linear_velocity.z = 0.0

		self.cf3.linear_acceleration.x = 0.0
		self.cf3.linear_acceleration.y = 0.0
		self.cf3.linear_acceleration.z = 0.0

		self.cf3.orientation.x = 0.0
		self.cf3.orientation.y = 0.0
		self.cf3.orientation.z = 0.0
		self.cf3.orientation.w = 1.0

		self.cf3.angular_velocity.x = 0.0
		self.cf3.angular_velocity.y = 0.0
		self.cf3.angular_velocity.z = 0.0

		self.cf3.angular_acceleration.x = 0.0
		self.cf3.angular_acceleration.y = 0.0
		self.cf3.angular_acceleration.z = 0.0



		self.rate = rospy.Rate(10)

		self.U = []
		self.X_position = []

		self.bias = []
		self.bias.append(np.array([0.5,0.5,0]))
		self.bias.append(np.array([-0.5,0.5,0]))
		self.bias.append(np.array([-0.5,-0.5,0]))
		self.bias.append(np.array([0.5,-0.5,0]))

		self.desires = []
		self.desires.append(np.array([1,1,1.5]))
		self.desires.append(np.array([1,1.5,1.5]))
		self.desires.append(np.array([1.5,1.5,1.5]))
		self.desires.append(np.array([1.5,1,1.5]))

		time.sleep(1.0)

		# Get current positions
		for n in range(1000):
			self.pos()
			# Calculate input
			self.controller(n)
			# Publish control signals
			self.cf0.position.x = self.U[0+n*4][0]
			self.cf0.position.y = self.U[0+n*4][1]
			self.cf0.position.z = self.U[0+n*4][2]
			print('x0'+str(self.x0))
			print('u0'+str(self.U[n*4]))

			self.cf1.position.x = self.U[1+n*4][0]
			self.cf1.position.y = self.U[1+n*4][1]
			self.cf1.position.z = self.U[1+n*4][2]
			print('x1'+str(self.x1))
			print('u1'+str(self.U[1+n*4]))


			self.cf2.position.x = self.U[2+n*4][0]
			self.cf2.position.y = self.U[2+n*4][1]
			self.cf2.position.z = self.U[2+n*4][2]
			print('x2'+str(self.x2))
			print('u2'+str(self.U[2+n*4]))
			#

			self.cf3.position.x = self.U[3+n*4][0]
			self.cf3.position.y = self.U[3+n*4][1]
			self.cf3.position.z = self.U[3+n*4][2]
			print('x3'+str(self.x3))
			print('u3'+str(self.U[3+n*4]))
			self.pub0.publish(self.cf0)
			self.pub1.publish(self.cf1)
			self.pub2.publish(self.cf2)
			self.pub3.publish(self.cf3)

			time.sleep(0.3)

			if n>400:
				self.desires[0]=self.X_position[0]
				self.desires[1]=self.X_position[1]
				self.desires[2]=self.X_position[2]
				self.desires[3]=self.X_position[3]
				self.bias[0]=np.array([0,0.75,0])
				self.bias[1]=np.array([0,0.25,0])
				self.bias[2]=np.array([0,-0.25,0])
				self.bias[3]=np.array([0,-0.75,0])

		time.sleep(2.0)


	def controller(self,n):
		bia0 = self.bias[1]+self.bias[2]+self.bias[3]-3*self.bias[0]
		u0 = 0.4*self.desires[0]+0.2*(self.X_position[1+n*4]+self.X_position[2+n*4]+self.X_position[3+n*4]+bia0)
		u0 = 0.0*self.x0+1.0*u0
		self.U.append(u0)

		# Agent 1
		bia1 = self.bias[0]+self.bias[2]+self.bias[3]-3*self.bias[1]
		u1 = 0.4*self.desires[1]+0.2*(self.X_position[0+n*4]+self.X_position[2+n*4]+self.X_position[3+n*4]+bia1)
		u1 = 0.0*self.x1+1.0*u1
		self.U.append(u1)

		# Agent 2
		bia2 = self.bias[1]+self.bias[0]+self.bias[3]-3*self.bias[2]
		u2 = 0.4*self.desires[2]+0.2*(self.X_position[1+n*4]+self.X_position[0+n*4]+self.X_position[3+n*4]+bia2)
		u2 = 0.0*self.x2+1.0*u2
		self.U.append(u2)

		# Agent 3
		bia3 = self.bias[1]+self.bias[2]+self.bias[0]-3*self.bias[3]
		u3 = 0.4*self.desires[3]+0.2*(self.X_position[1+n*4]+self.X_position[2+n*4]+self.X_position[0+n*4]+bia3)
		u3 = 0.0*self.x3+1.0*u3
		self.U.append(u3)


	def pos(self):
		self.X_position.append(self.x0)
		self.X_position.append(self.x1)
		self.X_position.append(self.x2)
		self.X_position.append(self.x3)


	def callback0(self,data):
		self.x0 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])


	def callback1(self,data):
		self.x1 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])


	def callback2(self,data):
		self.x2 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])


	def callback3(self,data):
		self.x3 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])

if __name__ == "__main__":
	try:
		cf_consensus()
	except rospy.ROSInterruptException:
		rospy.loginfo("Action terminated.")
