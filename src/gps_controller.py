#! /usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import pandas as pd

x = 0.0
y = 0.0
time=0.0

#store the current position of robot
def newNav (msg):
    global x
    global y
    global time

    x = msg.latitude
    y = msg.longitude
    time=msg.header.stamp

rospy.init_node ("gps_controller")

sub = rospy.Subscriber("/navsat/fix", NavSatFix, newNav)

TrackData=[]
while not rospy.is_shutdown():
    non=raw_input('press any button:')
    if non=='end':
        data=pd.DataFrame(TrackData)
        data.to_csv('trackdata.csv')
    else:
        print(x,y, time)
	#dummy= [a,b,c]
        TrackData=TrackData + [[x,y,time]]
