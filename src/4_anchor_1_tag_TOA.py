#!/usr/bin/env python

from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, get_first_pozyx_serial_port)
from pypozyx.tools.version_check import perform_latest_version_check
import numpy as np
import math

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
    toaB=1/2*(toaK2-toaK2[0]-toar2+toar[0]**2)
    # row delete
    toaH=toaH[1:,:]
    toaB=toaB[1:,:]
    # toa prediction
    TOApred=np.linalg.inv(toaH.T@toaH)@toaH.T@toaB
    return TOApred
'''
---------------------Kalman-----------------------------
'''
def TrackKalman(z):
    global A, Hk, Q, R, xk, P
    # kalman
    xp=A@xk
    Pp=A@P@A.T+Q
    K=Pp@Hk.T@np.linalg.inv(Hk@Pp@Hk.T+R)
    xk=xp+K@(z-Hk@xp)
    P=Pp-K@Hk@Pp
    #
    point=np.array([[xk[0]],[xk[2]]])
    return point
'''
-------------------TOA & Kalman setting-------------------
'''
A=np.array([[1,1,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
Hk=np.array([[1,0,0,0],[0,0,1,0]])
Q=np.eye(4)/200
R=0.25*np.eye(2)
xk=np.array([[0],[0],[0],[0]])
P=100*np.eye(4)
# anchor location setting AP2 AP3 AP4 AP5
Hnum1=([0,0],[4800,0],[4800,4800],[0,4800])
# anchor height
tag_height=1620-430
'''
--------------------------------------------------------
'''
if __name__ == "__main__":
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
    
    destination_id1 = 0x6739      #AP2
    destination_id2 = 0x672c      #AP3
    destination_id3 = 0x6758      #AP4
    destination_id4 = 0x677d      #AP5
    range_step_mm = 1000
    
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
        print(TOApred)

##        TOApred=toafunc(H,r)
##        x.append(TOApred[0])
##        y.append(TOApred[1])        
