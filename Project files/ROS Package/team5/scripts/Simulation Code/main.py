#!/usr/bin/env python3
import rospy
import math
import time
from std_msgs.msg import Float32, Int32
irleft=0
irright=0
irmiddle=0
proxsensor=0
speed=5.0000
hum_vision=0

def acallback(data):
    global proxsensor
    proxsensor=data.data

def xcallback(data):
    global irleft
    irleft=data.data
def ycallback(data):
    global irmiddle
    irmiddle=data.data
    #rospy.loginfo("Path completed succesfully")
    
def zcallback(data):
    global irright
    irright=data.data
    #rospy.loginfo("Path completed succesfully")
    
def vision(data):
    global hum_vision
    hum_vision=data.data
    #rospy.loginfo("Path completed succesfully")	
    

if __name__ == "__main__":

    rospy.init_node("irsensorreadings")
    rospy.Subscriber("/sim_ros_interface/ir_leftsensor/state", Float32, xcallback)
    rospy.Subscriber("/sim_ros_interface/ir_middlesensor/state", Float32, ycallback)
    rospy.Subscriber("/sim_ros_interface/ir_rightsensor/state", Float32, zcallback)
    rospy.Subscriber("/sim_ros_interface/proximitysensor/state", Float32, acallback)
    
    left_wheel_speed_pub = rospy.Publisher("/left_motor/setpoint_speed", Float32, queue_size=10)
    right_wheel_speed_pub = rospy.Publisher("/right_motor/setpoint_speed", Float32, queue_size=10)
    vision_pub = rospy.Publisher("/start/vision", Int32, queue_size=10)
    servo_pub = rospy.Publisher("/servo/setpoint_speed", Float32, queue_size=10)
    
    rate = rospy.Rate(10)
    time.sleep(3)
    
    while not rospy.is_shutdown():
            
        #rospy.loginfo("right reading is %f",irright)
        if irleft ==1.0 and irright ==1.0 and irmiddle ==0.0:
            leftmotorspeed=-1*speed
            rightmotorspeed=speed
            left_wheel_speed_pub.publish(leftmotorspeed)
            right_wheel_speed_pub.publish(rightmotorspeed)
            #rospy.loginfo("Path completed succesfully")
        if irleft ==0.0 and irright ==0.0:

            left_wheel_speed_pub.publish(0)
            right_wheel_speed_pub.publish(0)
            time.sleep(5)
            servo_pub.publish(60*3.14/180)
            time.sleep(9)
            rospy.loginfo("Checking right seat...")
            vision_pub.publish(1)
            time.sleep(5)
            vision_pub.publish(0)
            if hum_vision==1:
                rospy.loginfo("seat checked")	
                rospy.loginfo("Please scan your card...")
                rospy.loginfo("Waiting for passenger to scan...")
                rospy.loginfo("Thank you for scanning your card")
            else:
                rospy.loginfo("No human detected")
            
            time.sleep(10)
            servo_pub.publish(0)
            time.sleep(9)
            servo_pub.publish(-1*60*3.14/180)
            time.sleep(9)
            rospy.loginfo("Checking left seat...")
            if hum_vision==1:
                rospy.loginfo("seat checked")	
                rospy.loginfo("Please scan your card...")
                rospy.loginfo("Waiting for passenger to scan...")
                rospy.loginfo("Thank you for scanning your card")
            else:
                rospy.loginfo("No human detected")
        
            time.sleep(10)
            servo_pub.publish(0)
            time.sleep(9)
            left_wheel_speed_pub.publish(-5)
            right_wheel_speed_pub.publish(5)
            time.sleep(20)
            
            #rospy.loginfo("Path completed succesfully")
        if irright ==1.0 and irmiddle ==1.0 and irleft==0.0:
            leftmotorspeed=-1*speed*0.9
            rightmotorspeed=speed
            left_wheel_speed_pub.publish(leftmotorspeed)
            right_wheel_speed_pub.publish(rightmotorspeed)
            #rospy.loginfo("Path completed succesfully")
        if irleft ==1.0 and irmiddle ==1.0 and irright==0.0:
            leftmotorspeed=-1*speed
            rightmotorspeed=speed*0.9
            left_wheel_speed_pub.publish(leftmotorspeed)
            right_wheel_speed_pub.publish(rightmotorspeed)
            #rospy.loginfo("Path completed succesfully")
            
        if proxsensor==1:
            leftmotorspeed=0
            rightmotorspeed=0
            left_wheel_speed_pub.publish(leftmotorspeed)
            right_wheel_speed_pub.publish(rightmotorspeed)
            rospy.loginfo("Please clear the path")
            time.sleep(30)
            
            
        rate.sleep()

