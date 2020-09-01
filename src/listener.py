#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np

x=0.0
y=0.0

def callback1(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.data)
    global x
    
    x=data.data


def callback2(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.data)
    global y
    
    y=data.data

#def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber('chatter1', Int32, callback1)
    #rospy.Subscriber('chatter2', Int32, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

rospy.init_node('listener', anonymous=True)

sub1= rospy.Subscriber('/chatter1', Float32, callback1)
sub2= rospy.Subscriber('/chatter2', Float32, callback2)

r=rospy.Rate(4)
rospy.sleep(3)


while not rospy.is_shutdown():
    TOA=np.array([[0,0]])
    TOA[0][0]=x
    TOA[0][1]=y
    print(TOA)
