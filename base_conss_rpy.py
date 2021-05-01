#!/usr/bin/env python

import numpy as np
import rospy
import time
from mav_msgs.msg import DroneState # message
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, cos, sin, asinh, acosh
from tf.transformations import quaternion_from_euler, euler_from_quaternion


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

        # x boundary conditions 
        xv_start = 0                   # velocity (inital)
        xv_end = 0                     # velocity (next)
        xa_start = 0                   # acceleration (inital)
        xa_end = xa_start              # acceleration (next)
        xj_start = 0                   # jerk (inital)
        xj_end = xj_start              # jerk (next)

        # y boundary conditions
        yv_start = 0
        yv_end = 0
        ya_start = 0
        ya_end = ya_start
        yj_start = 0
        yj_end = yj_start

        # z boundary conditions 
        zv_start = 0
        zv_end = 0
        za_start = 0
        za_end = za_start
        zj_start = 0
        zj_end = zj_start

        # yaw boundary conditions
        yawv_start = 0
        yawv_end = 0
        yawa_start = 0
        yawa_end = 0
        yawj_start = 0
        yawj_end = 0

        # roll boundary conditions
        rollv_start = 0
        rollv_end = 0
        rolla_start = 0
        rolla_end = 0
        rollj_start = 0
        rollj_end = 0

        # pitch boundary conditions
        pitchv_start = 0
        pitchv_end = 0
        pitcha_start = 0
        pitcha_end = 0
        pitchj_start = 0
        pitchj_end = 0

        self.rate = rospy.Rate(10)

        self.U = []
        self.X_position = []
        self.X_orientation = []

        self.bias = []
        self.bias.append(np.array([0.5,0.5,0]))
        self.bias.append(np.array([-0.5,0.5,0]))
        self.bias.append(np.array([-0.5,-0.5,0]))
        self.bias.append(np.array([0.5,-0.5,0]))

        self.desires = []
        self.desires.append(np.array([1,1,1]))
        self.desires.append(np.array([1,1.5,1]))
        self.desires.append(np.array([1.5,1.5,1]))
        self.desires.append(np.array([1.5,1,1]))

        time.sleep(1.0)

        for n in range(500):
            # Update current positions
            self.pos()

            # Calculate input
            self.controller(n)

            # Publish control signals
            T = 0.1

        # CRAZYFLIE 0
            current_waypoint = self.X_position[0+n*4]
            next_waypoint = self.U[0+n*4]

            # x boundary conditions 
            xp_start = current_waypoint[0] # positon (inital)
            xp_end = next_waypoint[0]      # position (next)

            # y boundary conditions
            yp_start = current_waypoint[1]
            yp_end = next_waypoint[1]

            # z boundary conditions 
            zp_start = current_waypoint[2]
            zp_end = next_waypoint[2]

            current_orientation = self.X_orientation[0+n*4]
            current_angle = euler_from_quaternion(current_orientation)

            # roll boundary conditions
            rollp_start = current_angle[0]
            print((xp_end-xp_start))
            print((zp_end-zp_start))
            print((yp_end-yp_start))
            print((xp_end-xp_start)/(zp_end-zp_start))
            rollp_end = asinh((xp_end-xp_start)/(zp_end-zp_start))

            # pitch boundary conditions
            pitchp_start = current_angle[1]
            pitchp_end = acosh(abs(yp_end-yp_start)/abs(zp_end-zp_start))

            # yaw boundary conditions
            yawp_start = current_angle[2]
            yawp_end = atan2((yp_end-yp_start),(xp_end-xp_start))

            #compute parameters: 19 parameters, must be published 
            ax0 = self.polynomial_time_scaling_7th_order(xp_start, xp_end, xv_start, xv_end, xa_start, xa_end, xj_start, xj_end, T)
            ay0 = self.polynomial_time_scaling_7th_order(yp_start, yp_end, yv_start, yv_end, ya_start, ya_end, yj_start, yj_end, T)
            az0 = self.polynomial_time_scaling_7th_order(zp_start, zp_end, zv_start, zv_end, za_start, za_end, zj_start, zj_end, T)
            aroll0 = self.polynomial_time_scaling_7th_order(rollp_start, rollp_end, rollv_start, rollv_end, rolla_start, rolla_end, rollj_start, rollj_end, T)
            apitch0 = self.polynomial_time_scaling_7th_order(pitchp_start, pitchp_end, pitchv_start, pitchv_end, pitcha_start, pitcha_end, pitchj_start, pitchj_end, T)
            ayaw0 = self.polynomial_time_scaling_7th_order(yawp_start, yawp_end, yawv_start, yawv_end, yawa_start, yawa_end, yawj_start, yawj_end, T)


        # CRAZYFLIE 1
            current_waypoint = self.X_position[1+n*4]
            next_waypoint = self.U[1+n*4]

            # x boundary conditions 
            xp_start = current_waypoint[0] # positon (inital)
            xp_end = next_waypoint[0]      # position (next)

            # y boundary conditions
            yp_start = current_waypoint[1]
            yp_end = next_waypoint[1]

            # z boundary conditions 
            zp_start = current_waypoint[2]
            zp_end = next_waypoint[2]

            current_orientation = self.X_orientation[1+n*4]
            current_angle = euler_from_quaternion(current_orientation)

            # roll boundary conditions
            rollp_start = current_angle[0]
            rollp_end = asinh((xp_end-xp_start)/(zp_end-zp_start))

            # pitch boundary conditions
            pitchp_start = current_angle[1]
            pitchp_end = acosh(abs(yp_end-yp_start)/abs(zp_end-zp_start))

            # yaw boundary conditions
            yawp_start = current_angle[2]
            yawp_end = atan2((yp_end-yp_start),(xp_end-xp_start))

            #compute parameters: 19 parameters, must be published 
            ax1 = self.polynomial_time_scaling_7th_order(xp_start, xp_end, xv_start, xv_end, xa_start, xa_end, xj_start, xj_end, T)
            ay1 = self.polynomial_time_scaling_7th_order(yp_start, yp_end, yv_start, yv_end, ya_start, ya_end, yj_start, yj_end, T)
            az1 = self.polynomial_time_scaling_7th_order(zp_start, zp_end, zv_start, zv_end, za_start, za_end, zj_start, zj_end, T)
            aroll1 = self.polynomial_time_scaling_7th_order(rollp_start, rollp_end, rollv_start, rollv_end, rolla_start, rolla_end, rollj_start, rollj_end, T)
            apitch1 = self.polynomial_time_scaling_7th_order(pitchp_start, pitchp_end, pitchv_start, pitchv_end, pitcha_start, pitcha_end, pitchj_start, pitchj_end, T)
            ayaw1 = self.polynomial_time_scaling_7th_order(yawp_start, yawp_end, yawv_start, yawv_end, yawa_start, yawa_end, yawj_start, yawj_end, T)


        # CRAZYFLIE 2
            current_waypoint = self.X_position[2+n*4]
            next_waypoint = self.U[2+n*4]

            # x boundary conditions 
            xp_start = current_waypoint[0] # positon (inital)
            xp_end = next_waypoint[0]      # position (next)

            # y boundary conditions
            yp_start = current_waypoint[1]
            yp_end = next_waypoint[1]

            # z boundary conditions 
            zp_start = current_waypoint[2]
            zp_end = next_waypoint[2]

            current_orientation = self.X_orientation[2+n*4]
            current_angle = euler_from_quaternion(current_orientation)

            # roll boundary conditions
            rollp_start = current_angle[0]
            rollp_end = asinh((xp_end-xp_start)/(zp_end-zp_start))

            # pitch boundary conditions
            pitchp_start = current_angle[1]
            pitchp_end = acosh(abs(yp_end-yp_start)/abs(zp_end-zp_start))

            # yaw boundary conditions
            yawp_start = current_angle[2]
            yawp_end = atan2((yp_end-yp_start),(xp_end-xp_start))

            #compute parameters: 19 parameters, must be published 
            ax2 = self.polynomial_time_scaling_7th_order(xp_start, xp_end, xv_start, xv_end, xa_start, xa_end, xj_start, xj_end, T)
            ay2 = self.polynomial_time_scaling_7th_order(yp_start, yp_end, yv_start, yv_end, ya_start, ya_end, yj_start, yj_end, T)
            az2 = self.polynomial_time_scaling_7th_order(zp_start, zp_end, zv_start, zv_end, za_start, za_end, zj_start, zj_end, T)
            aroll2 = self.polynomial_time_scaling_7th_order(rollp_start, rollp_end, rollv_start, rollv_end, rolla_start, rolla_end, rollj_start, rollj_end, T)
            apitch2 = self.polynomial_time_scaling_7th_order(pitchp_start, pitchp_end, pitchv_start, pitchv_end, pitcha_start, pitcha_end, pitchj_start, pitchj_end, T)
            ayaw2 = self.polynomial_time_scaling_7th_order(yawp_start, yawp_end, yawv_start, yawv_end, yawa_start, yawa_end, yawj_start, yawj_end, T)


        # CRAZYFLIE 3
            current_waypoint = self.X_position[3+n*4]
            next_waypoint = self.U[3+n*4]

            # x boundary conditions 
            xp_start = current_waypoint[0] # positon (inital)
            xp_end = next_waypoint[0]      # position (next)

            # y boundary conditions
            yp_start = current_waypoint[1]
            yp_end = next_waypoint[1]

            # z boundary conditions 
            zp_start = current_waypoint[2]
            zp_end = next_waypoint[2]

            current_orientation = self.X_orientation[3+n*4]
            current_angle = euler_from_quaternion(current_orientation)

            # roll boundary conditions
            rollp_start = current_angle[0]
            rollp_end = asinh((xp_end-xp_start)/(zp_end-zp_start))

            # pitch boundary conditions
            pitchp_start = current_angle[1]
            pitchp_end = acosh(abs(yp_end-yp_start)/abs(zp_end-zp_start))

            # yaw boundary conditions
            yawp_start = current_angle[2]
            yawp_end = atan2((yp_end-yp_start),(xp_end-xp_start))

            #compute parameters: 19 parameters, must be published 
            ax3 = self.polynomial_time_scaling_7th_order(xp_start, xp_end, xv_start, xv_end, xa_start, xa_end, xj_start, xj_end, T)
            ay3 = self.polynomial_time_scaling_7th_order(yp_start, yp_end, yv_start, yv_end, ya_start, ya_end, yj_start, yj_end, T)
            az3 = self.polynomial_time_scaling_7th_order(zp_start, zp_end, zv_start, zv_end, za_start, za_end, zj_start, zj_end, T)
            aroll3 = self.polynomial_time_scaling_7th_order(rollp_start, rollp_end, rollv_start, rollv_end, rolla_start, rolla_end, rollj_start, rollj_end, T)
            apitch3 = self.polynomial_time_scaling_7th_order(pitchp_start, pitchp_end, pitchv_start, pitchv_end, pitcha_start, pitcha_end, pitchj_start, pitchj_end, T)
            ayaw3 = self.polynomial_time_scaling_7th_order(yawp_start, yawp_end, yawv_start, yawv_end, yawa_start, yawa_end, yawj_start, yawj_end, T)

            #publish parameters:
            
            t = 1/100
            for i in range(10):
                self.cf0 = self.move_to_point(ax0,ay0,az0,apitch0,aroll0,ayaw0,t,i)
                self.cf1 = self.move_to_point(ax1,ay1,az1,apitch1,aroll1,ayaw1,t,i)
                self.cf2 = self.move_to_point(ax2,ay2,az2,apitch2,aroll2,ayaw2,t,i)
                self.cf3 = self.move_to_point(ax3,ay3,az3,apitch3,aroll3,ayaw3,t,i)
                self.pub0.publish(self.cf0)
                self.pub1.publish(self.cf1)
                self.pub2.publish(self.cf2)
                self.pub3.publish(self.cf3)
                # time.sleep(t)
            

            # if n>400:
            #     self.desires[0]=self.X_position[0]
            #     self.desires[1]=self.X_position[1]
            #     self.desires[2]=self.X_position[2]
            #     self.desires[3]=self.X_position[3]
            #     self.bias[0]=np.array([0,0.75,0])
            #     self.bias[1]=np.array([0,0.25,0])
            #     self.bias[2]=np.array([0,-0.25,0])
            #     self.bias[3]=np.array([0,-0.75,0])
            self.rate.sleep()

        self.cf0 = self.stop_action(self.X_position[0+n*4])
        self.cf1 = self.stop_action(self.X_position[1+n*4])
        self.cf2 = self.stop_action(self.X_position[2+n*4])
        self.cf0 = self.stop_action(self.X_position[3+n*4])
        self.pub0.publish(self.cf0)
        self.pub1.publish(self.cf1)
        self.pub2.publish(self.cf2)
        self.pub3.publish(self.cf3)


    def controller(self,n):
        
        bia0 = self.bias[1]+self.bias[2]+self.bias[3]-3*self.bias[0]
        u0 = 0.4*self.desires[0]+0.2*(self.X_position[1+n*4]+self.X_position[2+n*4]+self.X_position[3+n*4]-bia0)
        u0 = 0.5*self.X_position[0+n*4]+0.5*u0
        self.U.append(u0)

        # Agent 1
        bia1 = self.bias[0]+self.bias[2]+self.bias[3]-3*self.bias[1]
        u1 = 0.4*self.desires[1]+0.2*(self.X_position[0+n*4]+self.X_position[2+n*4]+self.X_position[3+n*4]-bia1)
        u1 = 0.5*self.X_position[1+n*4]+0.5*u1
        self.U.append(u1)

        # Agent 2
        bia2 = self.bias[1]+self.bias[0]+self.bias[3]-3*self.bias[2]
        u2 = 0.4*self.desires[2]+0.2*(self.X_position[1+n*4]+self.X_position[0+n*4]+self.X_position[3+n*4]-bia2)
        u2 = 0.5*self.X_position[2+n*4]+0.5*u2
        self.U.append(u2)

        # Agent 3
        bia3 = self.bias[1]+self.bias[2]+self.bias[0]-3*self.bias[3]
        u3 = 0.4*self.desires[3]+0.2*(self.X_position[1+n*4]+self.X_position[2+n*4]+self.X_position[0+n*4]-bia3)
        u3 = 0.5*self.X_position[3+n*4]+0.5*u3
        self.U.append(u3)

    def stop_action(self, current_position):
        q = quaternion_from_euler(0,0,0)
        
        drone_msg = DroneState()
        drone_msg.position.x = current_position[0]
        drone_msg.position.y = current_position[1]
        drone_msg.position.z = current_position[2]
        drone_msg.linear_velocity.x = 0
        drone_msg.linear_velocity.y = 0
        drone_msg.linear_velocity.z = 0
        drone_msg.linear_acceleration.x = 0
        drone_msg.linear_acceleration.y = 0
        drone_msg.linear_acceleration.z = 0
        drone_msg.orientation.x = q[0]
        drone_msg.orientation.y = q[1]
        drone_msg.orientation.z = q[2]
        drone_msg.orientation.w = q[3]
        drone_msg.angular_velocity.x = 0
        drone_msg.angular_velocity.y = 0
        drone_msg.angular_velocity.z = 0
        drone_msg.angular_acceleration.x = 0
        drone_msg.angular_acceleration.y = 0
        drone_msg.angular_acceleration.z = 0

        return drone_msg



    def move_to_point(self,ax,ay,az,apitch,aroll,ayaw,t,i): 
        drone_msg = DroneState()

        # 7-order
        # x = ax[7] + ax[6]*(i*t) + ax[5]*((i*t)**2) + ax[4]*((i*t)**3) + ax[3]*((i*t)**4) + ax[2]*((i*t)**5) + ax[1]*((i*t)**6) + ax[0]*((i*t)**7)       
        # y = ay[7] + ay[6]*(i*t) + ay[5]*((i*t)**2) + ay[4]*((i*t)**3) + ay[3]*((i*t)**4) + ay[2]*((i*t)**5) + ay[1]*((i*t)**6) + ay[0]*((i*t)**7)      
        # z = az[7] + az[6]*(i*t) + az[5]*((i*t)**2) + az[4]*((i*t)**3) + az[3]*((i*t)**4) + az[2]*((i*t)**5) + az[1]*((i*t)**6) + az[0]*((i*t)**7)
        # yaw = ayaw[7] + ayaw[6]*(i*t) + ayaw[5]*((i*t)**2) + ayaw[4]*((i*t)**3) + ayaw[3]*((i*t)**4) + ayaw[2]*((i*t)**5) + ayaw[1]*((i*t)**6) + ayaw[0]*((i*t)**7) #position 
        
        # dx = ax[6] + 2.0*ax[5]*(i*t) + 3.0*ax[4]*((i*t)**2) + 4.0*ax[3]*((i*t)**3) + 5.0*ax[2]*((i*t)**4) + 6.0*ax[1]*((i*t)**5) + 7.0*ax[0]*((i*t)**6)
        # dy = ay[6] + 2.0*ay[5]*(i*t) + 3.0*ay[4]*((i*t)**2) + 4.0*ay[3]*((i*t)**3) + 5.0*ay[2]*((i*t)**4) + 6.0*ay[1]*((i*t)**5) + 7.0*ay[0]*((i*t)**6)
        # dz = az[6] + 2.0*az[5]*(i*t) + 3.0*az[4]*((i*t)**2) + 4.0*az[3]*((i*t)**3) + 5.0*az[2]*((i*t)**4) + 6.0*az[1]*((i*t)**5) + 7.0*az[0]*((i*t)**6)
        # dyaw = ayaw[6] + 2.0*ayaw[5]*(i*t) + 3.0*ayaw[4]*((i*t)**2) + 4.0*ayaw[3]*((i*t)**3) + 5.0*ayaw[2]*((i*t)**4) + 6.0*ayaw[1]*((i*t)**5) + 7.0*ayaw[0]*((i*t)**6) #velocity 
        
        # d2x = 2.0*ax[5] + 6.0*ax[4]*(i*t) + 12.0*ax[3]*((i*t)**2) + 20.0*ax[2]*((i*t)**3) + 30.0*ax[1]*((i*t)**4) + 42.0*ax[0]*((i*t)**5) 
        # d2y = 2.0*ay[5] + 6.0*ay[4]*(i*t) + 12.0*ay[3]*((i*t)**2) + 20.0*ay[2]*((i*t)**3) + 30.0*ay[1]*((i*t)**4) + 42.0*ay[0]*((i*t)**5)    
        # d2z = 2.0*az[5] + 6.0*az[4]*(i*t) + 12.0*az[3]*((i*t)**2) + 20.0*az[2]*((i*t)**3) + 30.0*az[1]*((i*t)**4) + 42.0*az[0]*((i*t)**5) 
        # d2yaw = 2.0*ayaw[5] + 6.0*ayaw[4]*(i*t) + 12.0*ayaw[3]*((i*t)**2) + 20.0*ayaw[2]*((i*t)**3) + 30.0*ayaw[1]*((i*t)**4) + 42.0*ayaw[0]*((i*t)**5) #acceleration

        x = ax[7] + ax[6]*(i*t) + ax[5]*((i*t)**2) + ax[4]*((i*t)**3)        
        y = ay[7] + ay[6]*(i*t) + ay[5]*((i*t)**2) + ay[4]*((i*t)**3)     
        z = az[7] + az[6]*(i*t) + az[5]*((i*t)**2) + az[4]*((i*t)**3) 
        roll = aroll[7] + aroll[6]*(i*t) + aroll[5]*((i*t)**2) + aroll[4]*((i*t)**3)
        pitch = apitch[7] + apitch[6]*(i*t) + apitch[5]*((i*t)**2) + apitch[4]*((i*t)**3)
        yaw = ayaw[7] + ayaw[6]*(i*t) + ayaw[5]*((i*t)**2) + ayaw[4]*((i*t)**3) #position 
        
        
        dx = ax[6] + 2.0*ax[5]*(i*t) + 3.0*ax[4]*((i*t)**2)
        dy = ay[6] + 2.0*ay[5]*(i*t) + 3.0*ay[4]*((i*t)**2)
        dz = az[6] + 2.0*az[5]*(i*t) + 3.0*az[4]*((i*t)**2)
        droll = aroll[6] + 2.0*aroll[5]*(i*t) + 3.0*aroll[4]*((i*t)**2)
        dpitch = apitch[6] + 2.0*apitch[5]*(i*t) + 3.0*apitch[4]*((i*t)**2)
        dyaw = ayaw[6] + 2.0*ayaw[5]*(i*t) + 3.0*ayaw[4]*((i*t)**2) #velocity 
        
        d2x = 2.0*ax[5] + 6.0*ax[4]*(i*t) 
        d2y = 2.0*ay[5] + 6.0*ay[4]*(i*t)    
        d2z = 2.0*az[5] + 6.0*az[4]*(i*t)
        d2roll = 2.0*aroll[5] + 6.0*aroll[4]*(i*t)
        d2pitch = 2.0*apitch[5] + 6.0*apitch[4]*(i*t) 
        d2yaw = 2.0*ayaw[5] + 6.0*ayaw[4]*(i*t) #acceleration

        # roll = atan2(-y, sqrt( (x*x) + (z*z) ) ) #atan2(y,z) 
        # pitch = atan2(x, sqrt( (y*y) + (z*z) ) ) #atan2(x,z) 
        q = quaternion_from_euler(roll, pitch, yaw)

        # Conver the angular velocity into B space
        rollDesRad_ = roll
        pitchDesRad_ = pitch
        yawDesRad_ = yaw
        roll_component = droll
        pitch_component = dpitch
        yaw_component = dyaw
        roll_component_B = roll_component - sin(pitchDesRad_) * yaw_component
        pitch_component_B = cos(rollDesRad_) * pitch_component + sin(rollDesRad_) * cos(pitchDesRad_) * yaw_component
        yaw_component_B = -sin(rollDesRad_) * pitch_component + cos(rollDesRad_) * cos(pitchDesRad_) * yaw_component



        # 19 commands: drone_msg --> position (x,y,z), linear_velocity (x,y,z), linear_acceleration(x,y,z)
        # orientation(x,y,z,w), angular_velocity(x,y,z), angular_acceleration(x,y,z)
        drone_msg.position.x = x
        drone_msg.position.y = y
        drone_msg.position.z = z
        drone_msg.linear_velocity.x = dx
        drone_msg.linear_velocity.y = dy
        drone_msg.linear_velocity.z = dz
        drone_msg.linear_acceleration.x = d2x
        drone_msg.linear_acceleration.y = d2y
        drone_msg.linear_acceleration.z = d2z
        drone_msg.orientation.x = q[0]
        drone_msg.orientation.y = q[1]
        drone_msg.orientation.z = q[2]
        drone_msg.orientation.w = q[3]
        drone_msg.angular_velocity.x = roll_component_B 
        drone_msg.angular_velocity.y = pitch_component_B
        drone_msg.angular_velocity.z = yaw_component_B
        drone_msg.angular_acceleration.x = d2roll
        drone_msg.angular_acceleration.y = d2pitch
        drone_msg.angular_acceleration.z = d2yaw
        
        return drone_msg


    def polynomial_time_scaling_7th_order(self, p_start, p_end, v_start, v_end, a_start, a_end, j_start, j_end, T):
        # input: p,v,a,j: position, velocity, acceleration and jerk of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        
        A = np.array([[   0,  0,  0,  0,  0,  0,  0,  1], \
                      [T**7, T**6, T**5, T**4, T**3, T**2, T, 1], \
                      [   0,  0,  0,  0,  0,  0,  1,  0], \
                      [7*(T**6), 6*(T**5), 5*(T**4), 4*(T**3), 3*(T**2), 2*T, 1, 0], \
                      [  0,  0,  0,  0,  0,  2,  0,  0], \
                      [42*(T**5), 30*(T**4), 20*(T**3), 12*(T**2), 6*T, 2, 0, 0], \
                      [  0,  0,  0,  0,  6,  0,  0,  0], \
                      [210*(T**4), 120*(T**3), 60*(T**2), 24*T, 6, 0, 0, 0]])

        A_inv = np.linalg.inv(A)

        v_end = 0.1*np.sign(p_end-p_start)

        boundaryCond = np.array([p_start, p_end, v_start, v_end, a_start, a_end, j_start, j_end])
        coefficients = np.dot(A_inv,boundaryCond)
        
        return coefficients



    def pos(self):
        self.X_position.append(self.x0)
        self.X_position.append(self.x1)
        self.X_position.append(self.x2)
        self.X_position.append(self.x3)

        self.X_orientation.append(self.angle0)
        self.X_orientation.append(self.angle1)
        self.X_orientation.append(self.angle2)
        self.X_orientation.append(self.angle3)


    def callback0(self,data):
        self.x0 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
        self.angle0 = np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])


    def callback1(self,data):
        self.x1 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
        self.angle1 = np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])


    def callback2(self,data):
        self.x2 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
        self.angle2 = np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])


    def callback3(self,data):
        self.x3 = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
        self.angle3 = np.array([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])


if __name__ == "__main__":
    try:
        cf_consensus()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")