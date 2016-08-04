#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math


def scan_callback(msg):
    running_sum = count_close = count_far = 0
    for r in msg.ranges[420:661]:
        if r < .16:
            running_sum += r
            count_close += 1
            count_far = 0
        else:
            count_far += 1
            if count_far == 30 and count_close:
                avg_r = running_sum / count_close
                #width = count_close / 1080 * 3/2 * math.pi * avg_r  # arc length
                #if width > .005:  # if width of object larger than 1 cm
                if count_close > 15:
                    #print "found chunk with angle width {} and distance {}".format(count_close, avg_r)
                    safety_pub.publish(go_back)
                    return
                running_sum = 0
                count_close = 0
    if count_close:  # check last window
        avg_r = running_sum / count_close
        #width = count_close / 1080 * 3/2 * math.pi * avg_r  # arc length
        #if width > .005:  # if width of object larger than 1 cm
        if count_close > 15:
            safety_pub.publish(go_back)


rospy.init_node("safety")
go_back = AckermannDriveStamped()
go_back.header.stamp = rospy.Time.now()
go_back.drive.steering_angle = 0
go_back.drive.speed = 0
safety_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)

rospy.spin()
