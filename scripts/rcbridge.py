#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket
import struct
import time
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
#from std_msgs.msg import UInt16MultiArray

# Byte format in message
packer = struct.Struct('B B h h h h h h h h h')

# Set header
hdr1ctrl = 84
hdr2ctrl = 1
hdr1fs = 70
hdr2fs = 80

# Channel values
ch = [0,0,0,0,0,0,0,0]
    
# Start up UDP socket
print 'RCBridge: Opening UDP Socket '
UDP_IP = "192.168.10.178"
UDP_PORT = 27200
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def sendpacket(data, fs_value):
    # Check control data is valid
    if len(data.data) == 8:
        # Form header
        if fs_value == 1:
            hdr1 = hdr1fs
            hdr2 = hdr2fs
        else:
            hdr1 = hdr1ctrl
            hdr2 = hdr2ctrl
            
        # Convert to values in 0 to 1000 range
        for i in range(0,8,1):
            ch[i] = int(data.data[i] * 1000)
            
        # Compute checksum
        chksum = hdr1+hdr2
        for chan in ch:
            chksum = chksum + (chan & 0xFF) + ((chan >> 8) & 0xFF)
            
        # Construct packet
        values = (hdr1,hdr2,ch[0],ch[1],ch[2],ch[3],ch[4],ch[5],ch[6],ch[7],chksum)
        UDPdata = packer.pack(*values)
        
        # print UDPdata
        
        # Send packet
        sock.sendto(UDPdata, (UDP_IP, UDP_PORT))
    else:
        print "Control data wrong length, should be 8"
    

def callback_ctrl(data):
    # Log incoming control message
    rospy.loginfo(rospy.get_caller_id()+"Inputs = %s",data.data)
    
    sendpacket(data, 0)
    
def callback_failsafe(data):
    # Log incoming failsafe message
    rospy.loginfo(rospy.get_caller_id()+"Failsafe = %s",data.data)
    
    sendpacket(data, 1)
    
def rcbridge():
    # Node naming
    rospy.init_node('rcbridge', anonymous=False) #anonymous=True)

    # Callbacks, e.g. incoming control messages
    rospy.Subscriber("ctrl", numpy_msg(Floats), callback_ctrl)
    rospy.Subscriber("failsafe", numpy_msg(Floats), callback_failsafe)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    rcbridge()
