#! /usr/bin/env python3
import rclpy
#import rospy
from rclpy.node import Node
import serial
import time
import sys
from threading import Thread

#from launch import LaunchDescription
from termios import tcflush, TCIFLUSH

#from kbot.msg import KBot
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import joint_state_publisher
import tf2_ros
import math
from math import cos, sin, pi
import numpy as np

import imu_ix as imu_
import send_ix as tx_

imu_.sample_time_ = 0.0083
imu_.sample_rate_ = 1 / imu_.sample_time_

imu_.mag_bias  = [0.061798095703125, 0.10134887695312494, 0.1717987060546875]
imu_.gyro_bias = [0.014114753777375467, 0.010852381442038685, -0.00665790272517]

imu_.i2c_bus_init()

imu_.imu_init()


ser       = ''
port      = ''
baud_rate = 2500000


right_wheel_w = 0
left_wheel_w  = 0    
str_ = ''

entry_count = 0



def send_cmd_ix(right_w, left_w):

    tcflush(sys.stdin, TCIFLUSH)

    global ser
    global entry_count

    cmd_ = "-v "  + str(int(right_w)) + " " + str(int(left_w)) + " \n"

    print("\n")
    print("\033[92m {}\033[00m".format(cmd_))
    print("\033[92m {}\033[00m".format("Right (deg/sec) : " + str(int(right_w))))
    print("\033[92m {}\033[00m".format("Left  (deg/sec) : " + str(int(left_w ))))
    print("\n")


    try:
        if(ser == ''):
            ser = serial.Serial(port, baud_rate, timeout = 0.15, write_timeout = 0.25)
        
        try:
            if(entry_count >= 1000 and right_w == 0 and left_w == 0):
                cmd_ = "-S\n"
            #ser.write(cmd_.encode('utf-8'))
            #ser.reset_input_buffer()
            #dummy = ser.read(1)
            read_data_ix()
            ser.write(cmd_.encode('utf-8'))
            #dummy = ser.read(100)
            #dummy = ser.read(10000000)
            #ser.reset_output_buffer()
            #ser.cancel_write()
            
            
        except Exception as e:
            print("\033[93m {}\033[00m".format(str(e) + " Can't Send Command To KBOT"))
            print("\n")
            return
    
    except Exception as e:
        print("\033[91m {}\033[00m".format(str(e) + " Can't Connect to KBOT"))
        print("\n")
        
    return
    


  
    
def read_data_ix():
    global ser
    global str_
    global right_wheel_w
    global left_wheel_w
    try:
        if(ser == ''):
           ser = serial.Serial(port, baud_rate, timeout = 0.15, write_timeout = 0.25)
    except Exception as e:
        print("\033[91m {}\033[00m".format(str(e)))
        print("\n")



    try:
            str_ = ser.read(ser.in_waiting)
            #str_ = ser.read(100)
            #str_ = ser.read_all()
            #ser.cancel_read()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            if(str_):
                str_ = str_.decode('utf-8')
                if(("MA :" in str_) and ("MB :" in str_) and ("\r" in str_)):
                    right_wheel_w = float(str_[str_.index("MA :") + 5 : str_.index("MB :")])
                    left_wheel_w  = float(str_[str_.index("MB :") + 5 : str_.index("\r")])
            
            
    except Exception as e:
            print("\033[93m {}\033[00m".format(str(e) + " Can't Read Wheel Angular Rate From KBOT"))
            print("\n")
            return False
        
    return (right_wheel_w, left_wheel_w)


       


class KBOT_ROVER_IX(Node):
    str_ = ''
    i_   =  0
    duration_sub = 0.000001
    duration_imu = 0.000005

    left_wheel_bias  = 1
    right_wheel_bias = 1
    wheel_radius     = 0.06  #in meter
    
    right_wheel_deg_ps = 0
    left_wheel_deg_ps  = 0
    
    right_dps = 0
    right_dps = 0

    odom_publish_rate = 0.04

    x   = 0.0
    y   = 0.0
    th  = 0.0
    th_ = 0.0

    wheel_right_pos = 0
    wheel_left_pos  = 0

    imu_data = [0.0, 0.0, 0.0]

    def calc_odom(self):
        global right_wheel_w
        global left_wheel_w
        
        self.current_time = self.get_clock().now()
        
        read_data_ix()

        print("\033[93m {}\033[00m".format("MA : " + str(right_wheel_w) + "   MB : " + str(left_wheel_w)))


        Vr = (right_wheel_w * 2 * pi * self.wheel_radius) / 360  #wheel radius 0.06m
        Vl = (left_wheel_w  * 2 * pi * self.wheel_radius) / 360

        vx =  (Vr + Vl) / 2              #
        vy =  0.0                        #0 since this is a Diif Drive Robot
        vth = (Vr - Vl) / (2 * 0.2875)   #0.2875m distance between 2 wheels

        # compute odometry in a typical way given the velocities of the robot
        #dt = (current_time - last_time).to_sec()
        dt = (self.current_time - self.last_time).nanoseconds / 1000000000
        delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
        delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th_ += delta_th   # Angle From Odom
        print("ANGLE : ", int(self.imu_data[2]))
        self.th = -1 * int(self.imu_data[2]) * pi / 180  # Angle From IMU

        #create odom
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        #set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        #set the orientation
        quaternion = Quaternion()
        quaternion.z = sin(self.th/ 2.0)
        quaternion.w = cos(self.th/ 2.0)
        odom.pose.pose.orientation = quaternion

        #setting up twist (linear / angular) velocity
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = vth #vth : Angular Rate From Odom

        #publish odom
        self.pub.publish(odom)
        self.last_time = self.current_time
        
        
        #joint states
        joint_state = JointState()
        self.wheel_right_pos += right_wheel_w * dt * (2 * pi / 360)
        self.wheel_left_pos  += left_wheel_w  * dt * (2 * pi / 360)
        #joint_state.header.frame_id = 'base_link'
        joint_state.header.stamp    = self.current_time.to_msg()
        joint_state.name     = ['left_wheel_r_joint', 'right_wheel_r_joint','left_wheel_f_joint','right_wheel_f_joint']
        joint_state.position = [self.wheel_left_pos, self.wheel_right_pos, self.wheel_left_pos, self.wheel_right_pos]

        #publish joint states
        self.pub_js.publish(joint_state)


        #create transform message
        transform_msg = TransformStamped()
        transform_msg.header.stamp    = self.current_time.to_msg()
        transform_msg.header.frame_id = 'odom'
        transform_msg.child_frame_id  = 'base_link'
        
        #set translation
        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = 0.0
        
        #set rotation
        transform_msg.transform.rotation = quaternion

        #broadcast transform
        self.tf_broadcaster.sendTransform(transform_msg)


    def sub_callback(self, msg):
        
        #Convert Linear / Angular Velocities To Wheel Angular Rate (deg/sec)
        right_vel = msg.linear.x + (msg.angular.z * (self.right_wheel_bias * 0.5))
        left_vel  = msg.linear.x - (msg.angular.z * (self.left_wheel_bias  * 0.5))

        self.right_wheel_deg_ps = (right_vel * 360) / (2 * pi * self.wheel_radius)
        self.left_wheel_deg_ps  = (left_vel  * 360) / (2 * pi * self.wheel_radius)

        send_cmd_ix(self.right_wheel_deg_ps, self.left_wheel_deg_ps)

        #self.get_logger().info('Received x linear  : "%s"' % msg.linear.x)
        #self.get_logger().info('Received z angular : "%d"' % msg.angular.z)

        
    def timer_callback(self):
    
        tcflush(sys.stdin, TCIFLUSH)
        send_cmd_ix(self.right_wheel_deg_ps, self.left_wheel_deg_ps)
        #send_cmd_ix(0, 0)


    def imu_read_(self):
        self.imu_data = imu_.imu_fusion_()
        #print(self.imu_data)
        print("\033[92m {}\033[00m".format(self.imu_data[2]))


    def __init__(self):
        super().__init__('kbot_rover_ix')

        self.current_time = self.get_clock().now()
        self.last_time    = self.get_clock().now()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.sub_callback, 2)
        
        self.pub    = self.create_publisher(Odometry, 'odom', 2)
        
        self.pub_js = self.create_publisher(JointState, 'joint_states', 2)
        
        thread_ = Thread(target = self.create_timer, args=(self.odom_publish_rate, self.calc_odom,))
        thread_.start()

        thread_imu = Thread(target = self.create_timer, args=(self.duration_imu, self.imu_read_,))
        thread_imu.start()

        self.sub
        
        self.timer = self.create_timer(self.duration_sub, self.timer_callback)
        thread_.join()
        thread_imu.join()

    
def main(args = None):
    rclpy.init(args = args)

    print("\033[92m {}\033[00m".format("INIT..."))

    node_ = KBOT_ROVER_IX()

    try:
        rclpy.spin(node_)
    except KeyboardInterrupt:
        pass
    

    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    try:
        port = "/dev/" + str(sys.argv[1])
    except:
        print("\033[93m {}\033[00m".format("No Serial Port Specified"))
        print("\033[92m {}\033[00m".format("Usage : ros2 run kbot kbot_rover.py <PORT>"))
        exit()

    main()
    
