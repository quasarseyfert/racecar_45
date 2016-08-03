#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from racecar_45.msg import speed as SpeedMsg
from std_msgs.msg import Int32MultiArray, String
from ackermann_msgs.msg import AckermannDriveStamped



class speed_control():
    def __init__(self):
	self.driving = AckermannDriveStamped()
	self.speedy = rospy.Subscriber("/speeds", SpeedMsg, self.speedcomptroller)
	self.pid_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
	self.color_type = rospy.Subscriber("/exploration_challenge",String,self.what_color,queue_size=10)#what should queue_size be???
	self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
	self.rightScan = []
	self.leftScan = []
	self.frontScan=[]
	self.topSpeed =2 
	self.color = None
	#self.previousspeed=0
	#self.acc = .1
    def what_color(self,msg):
    	color = msg
    def laser_callback(self,msg):
    	self.rightScan = msg.ranges[x:y]
	self.leftScan = msg.ranges[x:y]  # use values from charles PID
	self.frontScan = msg.ranges[x:y]
    def speedcomptroller(self,msg):
	#diff = (msg.speed - self.previousspeed)
	#if diff > 0: mult=1
	#else: mult = -1	

	#if abs(msg.speed) < .05:
	#    self.driving.drive.speed = msg.speed
	#elif abs(diff) > self.acc:
	#    self.driving.drive.speed = self.previousspeed + mult*self.acc
	#else:
	#    self.driving.drive.speed = msg.speed
	#self.previousspeed = self.driving.drive.speed
	#print (self.driving.drive.speed, msg.angle)
	if color == None:drive=True
	if drive:
	    self.driving.drive.steering_angle = msg.angle
	    self.driving.drive.speed = min(self.topSpeed,msg.speed)
	    self.pid_pub.publish(self.driving)
	elif color =="green":## might have reversed red/green this is for going straight
	    ##pid left wall (not online copy from robot)
	    drive = True
	    tooFar =0
	    for i in range(len(self.rightScan)):
	    	if rightScan[i] > 2:
	    	    tooFar +=1
	    	if tooFar>6:
	    	    drive = False
	    	    break
	elif color == "red":
	    Kp = 1.3
	    Kd = .001
	    ##PID
	    # sort left and right 
	    leftL= sum(sorted(self.leftScan)[6:16])/10
	    rightL= sum(sorted(self.rightScan)[6:16])/10
	    
	    if (leftL+rightL)< 1.5 and (leftL+rightL) > .9 and sum(self.frontScan)/len(self.frontScan) <2:
	    	drive = True
	    else:
	    	drive = False
	    

if __name__ == '__main__':
    rospy.init_node('roadrunner')
    node = speed_control()
    rospy.spin()
