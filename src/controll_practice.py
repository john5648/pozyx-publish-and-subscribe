#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi

x = 0.0
y = 0.0
theta = 0.0

#store the current position of robot
def newOdom (msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def angle(rotate_angle):
    global x
    global y
    global theta
    global flag

    target_radian= theta+rotate_angle
    if target_radian > pi:
        target_radian=target_radian-2*pi

    speed.linear.x = 0.0
    while abs(target_radian-theta)>0.1:
        speed.angular.z = 0.4
        pub.publish(speed)
        r.sleep()
    flag=1

def drive(goal_distance):
    global x
    global y
    global flag

    speed.angular.z = 0.0
    flag_2=0
    last_pos=[x,y]
    goal_distance=goal_distance**2

    while((x-last_pos[0])**2+(y-last_pos[1])**2 < goal_distance):
        speed.linear.x = 0.5
        if flag_2%6==5:
            speed.angular.z = -0.03
        else:
            speed.angular.z = 0.0

        pub.publish(speed)      
        r.sleep()
        flag_2=flag_2+1
    flag=0

def ro_cir():
    global x
    global y
    global flag

    speed.linear.x = 0.2
    speed.angular.z = 0.4

    while(True):
        pub.publish(speed)      
        r.sleep()

rospy.init_node ("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)

speed = Twist()

#sleeping rate 
r=rospy.Rate(4)

flag=3
i=0
tar_angle=pi*51/100 #0.51
rospy.sleep(3)

current_loc=[0,0]
target_loc=[4000,4000]

while not rospy.is_shutdown():
#    print(x)
    if flag==0:
        angle(tar_angle)
    elif flag==1:
        drive(7.2)
    elif flag==3:
        ro_cir()
