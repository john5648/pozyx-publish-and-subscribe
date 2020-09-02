#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check
import numpy as np
import math

import rospy
from std_msgs.msg import Float32

class ReadyToRange(object):
    def __init__(self, pozyx, destination_id, range_step_mm=1000, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION,
                 remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.range_step_mm = range_step_mm
        self.remote_id = remote_id
        self.protocol = protocol
    def setup(self):
        # set the ranging protocol
        self.pozyx.setRangingProtocol(self.protocol, self.remote_id)
    def loop(self):
        device_range = DeviceRange()
        status = self.pozyx.doRanging(
            self.destination_id, device_range, self.remote_id)
        dis=device_range.distance
        return dis

def talker1(ttt):
    pub1 = rospy.Publisher('chatter1', Float32, queue_size=10)
    rospy.init_node('talker1', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo(ttt)
    pub1.publish(ttt)
#    while not rospy.is_shutdown():
#        hello_str = ttt % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()

def talker2(tttt):
    pub2 = rospy.Publisher('chatter2', Float32, queue_size=10)
    #rospy.init_node('talker2', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo(tttt)
    pub2.publish(tttt)
'''
----------------------TOA-------------------------------
'''
def toafunc(H,r):
    c=3*pow(10,8)
    # H
    dummy=np.zeros(H.shape)
    dummy=dummy+H[0]
    toaH=H-dummy;
    # K**2
    toaK2=(sum((H*H).T)).reshape(H.shape[0],1)
    # r
    toar=r
    toar2=(sum((toar*toar).T)).reshape(toar.shape[0],1)
    # b
    toaB=0.5*(toaK2-toaK2[0]-toar2+toar[0]**2)
    # row delete
    toaH=toaH[1:,:]
    toaB=toaB[1:,:]
    # toa prediction
    TOApred=np.linalg.inv(toaH.T.dot(toaH)).dot(toaH.T).dot(toaB)
    return TOApred
'''
-------------------TOA & Kalman setting-------------------
'''
# anchor location setting AP2 AP3 AP4 AP5
#Hnum1=([0,0],[4800,0],[4800,4800],[0,4800])
Hnum1=([7540,7210],[7540,0],[14140,0],[14140,7580])
# anchor height
#tag_height=1620-430
tag_height=1970-430

if __name__ == '__main__':
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x676d          
    remote = True               
    if not remote:
        remote_id = None
    
    # destination_id1 = 0x6739      #AP2
    # destination_id2 = 0x672c      #AP3
    # destination_id3 = 0x6758      #AP4
    # destination_id4 = 0x677d      #AP5
    range_step_mm = 1000

    destination_id1 = 0x6714      #AP9
    destination_id2 = 0x6758      #AP2
    destination_id3 = 0x6a32      #AP3
    destination_id4 = 0x6e6e      #AP4   

    ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION

    pozyx = PozyxSerial(serial_port)
    range1 = ReadyToRange(pozyx, destination_id1, range_step_mm,
                     ranging_protocol, remote_id)
    range2 = ReadyToRange(pozyx, destination_id2, range_step_mm,
                     ranging_protocol, remote_id)
    range3 = ReadyToRange(pozyx, destination_id3, range_step_mm,
                     ranging_protocol, remote_id)
    range4 = ReadyToRange(pozyx, destination_id4, range_step_mm,
                     ranging_protocol, remote_id)    
    range1.setup()
    while True:
        r=[0]*4
        H=list(Hnum1)
        r[0]=range1.loop()
        r[1]=range2.loop()
        r[2]=range3.loop()
        r[3]=range4.loop()

        for i in range(0,len(r)):
            r[i]=math.sqrt(abs(r[i]**2-tag_height**2))
        
        if r.count(0) >=2:
            continue
        elif r.count(0)==1:
            del(H[r.index(0)])
            r.remove(0)  
        TOApred=toafunc(np.array(H[0:3]),np.array(r[0:3]).reshape(3,1))
        TOApred=TOApred.reshape(1,2)
        #print(TOApred)
    	talker1(TOApred[0][0])
	talker2(TOApred[0][1])

