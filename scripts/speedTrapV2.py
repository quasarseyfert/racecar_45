#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from racecar_45.msg import speed as SpeedMsg
from std_msgs.msg import Int32MultiArray, String
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import time, collections



class speed_control():
    def __init__(self):
	self.driving = AckermannDriveStamped()
	self.pid_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
	self.rightScan = []
	self.leftScan = []
	self.frontScan=[]
	self.topSpeed =2 
	self.color = ""
	self.color_type = rospy.Subscriber("/color_detection",String,self.what_color,queue_size=1)#what should queue_size be???
	self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)
	self.speedy = rospy.Subscriber("/speeds", SpeedMsg, self.speedcomptroller)
	#self.previousspeed=0
	#self.acc = .1
	self.prev_times = collections.deque([time.clock() for _ in range(10)])
        self.prev_errors = collections.deque([0 for _ in range(4)])
	self.drive = True
	self.isGreen = False
	self.isRed = False
	self.gCount = 0
	self.rCount = 0
    def what_color(self,msg):
    	self.color = msg.data
	if self.color == "red":
	    self.rCount+=1
	    self.gCount = 0
	elif self.color == "green":
	    self.gCount +=1
	    self.rCount = 0
	else:
	    self.gCount = 0
	    self.rCount = 0
	if not self.isGreen:
	    self.isGreen = self.gCount >= 5
	if not self.isRed:
	    self.isRed = self.rCount >=5
	self.drive = not self.color
    def laser_callback(self,msg):
	left_start_ind = 580
        left_end_ind = 1000
        right_start_ind = 80
        right_end_ind = 500
	front_start_ind =480
	front_end_ind = 600
	self.leftScan = msg.ranges[left_start_ind:left_end_ind]    	
	self.rightScan = msg.ranges[right_start_ind:right_end_ind]
	self.frontScan = msg.ranges[front_start_ind:front_end_ind]
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
	print(self.color)
	print(self.drive)
	if self.drive:
	    print "simply republishing"
	    self.driving.drive.steering_angle = msg.angle
	    self.driving.drive.speed = min(self.topSpeed,msg.speed)
	elif self.isGreen:
	    print "green"
	    kp = 1.5 #.5
            ki = .15 #.15
            kd = .05 #.05
	    ddes = .5
	    lDist = sum(sorted(self.leftScan)[6:16])/10.0  
	    error = ddes - lDist
	    if abs(error)<.03:
		self.driving.drive.steering_angle = 0
	    else:
           	self.driving.drive.steering_angle = self.pid(kp, kd, ki, error)
	    self.drive = True
	    tooFar =0
	    for i in range(len(self.rightScan)):
	    	if self.rightScan[i] > 4:
	    	    tooFar +=1
	    	if tooFar>6:
	    	    self.drive = False
		    self.isGreen=False
	    	    break
	elif self.isRed == "red":
	    print "red"
	    kp = 3
	    kd = .05
	    ki = 0
	    ddes = .6
	    rDist = sum(sorted(self.rightScan)[6:16])/10.0
	    error = ddes - rDist
	    if abs(error)<.03:
		self.driving.drive.steering_angle = 0
	    else:
           	self.driving.drive.steering_angle = self.pid(kp, kd, ki, error)
	    print self.driving.drive.steering_angle
	    ##PID
	    # sort left and right 
	    leftL= sum(sorted(self.leftScan)[6:16])/10
	    rightL= sum(sorted(self.rightScan)[6:16])/10
	    self.drive = (leftL+rightL)< 2.3 and (leftL+rightL) > 1.5 and sum(self.frontScan)/len(self.frontScan)<3.5
	    self.isRed== (leftL+rightL)< 2.3 and (leftL+rightL) > 1.5 and sum(self.frontScan)/len(self.frontScan)<3.5
	    if self.drive:
		print "turning off right pid"
	else:
	    print "simply republishing"
	    self.driving.drive.steering_angle = msg.angle
	    self.driving.drive.speed = min(self.topSpeed,msg.speed)
	self.pid_pub.publish(self.driving)
    def pid(self, kp, kd, ki, error):
        prev_error = self.prev_errors.popleft()
        prev_time = self.prev_times.popleft()
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        self.prev_times.append(time.clock())
        self.prev_errors.append(error)
        return kp * error + kd * e_deriv + ki * e_int	    

if __name__ == '__main__':
    rospy.init_node('roadrunner')
    node = speed_control()
    rospy.spin()
