#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Twist
from math import pi, sqrt, acos, degrees
import numpy as np

pozyx_x=0.0
pozyx_y=0.0

def callback1(data):
    global pozyx_x
    pozyx_x=data.data
def callback2(data):
    global pozyx_y
    pozyx_y=data.data

def angle(rotate_angle, rot_dir):
    global flag
    #rotate speed is 30
    angular_speed = 30*2*pi/360
    relative_angle = rotate_angle*2*pi/360
    current_angle = 0.0

    speed.linear.x = 0.0
    if rot_dir>=0:
        speed.angular.z = abs(angular_speed)
    elif rot_dir<0:
        speed.angular.z = -abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    # multplying 1.158 is to correct the real world angle
    while(current_angle < relative_angle*1.158):
        pub.publish(speed)
        r.sleep()
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)
    r.sleep()

def along_line(current_loc, target_loc):
    global flag   
    #time_counter lets jackal turn angular.z=0.4 for only 10 times 
    # and 20 times with angular.z=0.1 to prevent overshooting
    speed.angular.z = 0.0
    speed.linear.x = 0.0
    criteriion_dis=sqrt((current_loc[0]-target_loc[0])**2 + (current_loc[1]-target_loc[1])**2)
    while(True):
        [eqt, remain_dis, drive_dis, sign_num] = criterion(current_loc, target_loc)
        if eqt >= 100:
            speed.linear.x = 0.4
            speed.angular.z = -0.1*sign_num
        elif eqt < -100:
            speed.linear.x = 0.4
            speed.angular.z = 0.1*sign_num
        else:
            speed.linear.x = 0.4
            speed.angular.z = 0.0            
        if remain_dis<200 or drive_dis>=criteriion_dis:
            break
        #print(eqt, speed.angular.z, sign_num)
        pub.publish(speed)      
        r.sleep()   
        
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)
    r.sleep()  
    flag=1
    
def criterion(current_loc, target_loc):
    global pozyx_x
    global pozyx_y

    if target_loc[0]-current_loc[0] != 0:
        linear_eqt=pozyx_y-current_loc[1]-(pozyx_x-current_loc[0])*(target_loc[1]-current_loc[1])/(target_loc[0]-current_loc[0])
    else:
        linear_eqt=pozyx_x-target_loc[0]

    if target_loc[0]-current_loc[0]>=0:
        signed_num=1
    else:
        signed_num=-1

    remain_dis= sqrt((target_loc[0]-pozyx_x)**2 + (target_loc[1]-pozyx_y)**2)
    drive_dis= sqrt((current_loc[0]-pozyx_x)**2 + (current_loc[1]-pozyx_y)**2)

    return linear_eqt, remain_dis, drive_dis, signed_num

def finding_slope(target_point):
    base_point=np.array([pozyx_x, pozyx_y])
    t0 = rospy.Time.now().to_sec()
    while(rospy.Time.now().to_sec()-t0<3):
        speed.linear.x = 0.3
        speed.angular.z = 0.0
        pub.publish(speed)
        r.sleep()
    
    # stop the Jackal
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)
    r.sleep()

    arrived_point=np.array([pozyx_x, pozyx_y])
    #print(base_point,arrived_point,target_point)
    #making vectors unit vector
    slope_vector=np.subtract(arrived_point,base_point)
    target_vector=np.subtract(target_point,arrived_point)
    #print(slope_vector, target_vector)
    slope_vector=slope_vector/np.linalg.norm(slope_vector)
    target_vector=target_vector/np.linalg.norm(target_vector)
    # inner product and cross product of two vector
    inner_angle=degrees(acos(np.inner(slope_vector, target_vector)))
    cross_product=np.cross(slope_vector,target_vector)
    
    #print(slope_vector,target_vector,inner_angle)
    return inner_angle, cross_product, arrived_point

rospy.init_node('listener', anonymous=True)
sub1= rospy.Subscriber('/chatter1', Float32, callback1)
sub2= rospy.Subscriber('/chatter2', Float32, callback2)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)

speed = Twist()

#sleeping rate 
r=rospy.Rate(10) #10hz

flag=0
#pozyx need some time to set up
rospy.sleep(3)

jackal_loc=np.array([[11140,1200]])
#jackal_loc=np.array([[0,2400],[3000,2400]])
slot=0
while not rospy.is_shutdown():
    if flag==0:
        print('entering slope')
        [inner, outer, current_position]=finding_slope(jackal_loc[0])
        flag=1
    elif flag==1:
        print('entering angle')
        angle(inner, outer)
        flag=2
    elif flag==2:
        print('entering drive')
        along_line(current_position, jackal_loc[0])
        flag=3
    elif flag==3:
        break
    # if flag==0:
    #     angle(90,1)
    #     break
    # elif flag==1:
    #     along_line(jackal_loc[slot], jackal_loc[slot+1])
    # elif flag==2:
    #     print('done')
    # elif flag==3:
    #     i=0
    #     ii=0
    #     while(i<30):
    #         speed.linear.x = i
    #         pub.publish(speed)
    #         r.sleep()
    #         ii=ii+1
    #         if ii%2==0:
    #             i=i+0.02
    #     speed.linear.x = 0
    #     pub.publish(speed)
    #     r.sleep()
    
    # slot=slot+1
    # if slot>=len(jackal_loc)-1:
    #     break

