#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket
import struct
import time
import random
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
# from std_msgs.msg import UInt16MultiArray

# Byte format in message
packer = struct.Struct('B B h h h h h h h h h')

# Set header
hdr1ctrl = 84
hdr2ctrl = 1
hdr1fs = 70
hdr2fs = 80

# Channel values
ch = [0, 0, 0, 0, 0, 0, 0, 0]

# Node naming
rospy.init_node('quadrcbridge', anonymous=False)  # anonymous=True)

# Start up UDP socket
# IP address and port from parameters, or use defaults
UDP_IP = rospy.get_param('~ipaddr', "192.168.10.173")
UDP_PORT = rospy.get_param('~udpport', 27200)
rospy.loginfo('Opening UDP Socket to %s : %i', UDP_IP, UDP_PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# send set of eight channel settings to the Arduino/RC bridge
# if fs_value=1, these are the failsafe settings
# otherwise they are operational settings
# data is a list of 8 values
def sendpacket(data, fs_value):
  # debug
  # rospy.loginfo('Trying to send %s', data)
  # Check control data is valid
  if 1:
    # Form header
    if fs_value == 1:
      hdr1 = hdr1fs
      hdr2 = hdr2fs
    else:
      hdr1 = hdr1ctrl
      hdr2 = hdr2ctrl

    # Convert to values into 0 to 1000 range
    if type(data) == Twist:
      # Populate channels, order is roll, pitch, thrust, yaw
      ch[0] = int(data.linear.y*500 + 500)
      ch[1] = int(data.linear.x*500 + 500)
      ch[2] = int(data.linear.z*500 + 500)
      ch[3] = int(data.angular.z*500 + 500)

      # Saturate between 0 and 1000
      for i in range(0, 4, 1):
        if ch[i] < 0:
          ch[i] = int(0)
        if ch[i] > 1000:
          ch[i] = int(1000)

      # Send neutral on remaining four channels
      for i in range(4, 8, 1):
        ch[i] = int(500)

    elif type(data) == list:
      for i in range(0,8,1):
        ch[i] = int(data[i] * 1000)
    else:
      return

    # Compute checksum
    chksum = hdr1 + hdr2
    for chan in ch:
      chksum = chksum + (chan & 0xFF) + ((chan >> 8) & 0xFF)

    # Construct packet
    values = (hdr1, hdr2, ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], chksum)
    UDPdata = packer.pack(*values)

    # print UDPdata

    # Send packet
    sock.sendto(UDPdata, (UDP_IP, UDP_PORT))

def callback_ctrl(data):
  # Log incoming control message
  rospy.loginfo(rospy.get_caller_id() + "Inputs = [%5.2f %5.2f %5.2f %5.2f]", data.linear.x, data.linear.y, data.linear.z, data.angular.z)
  sendpacket(data, 0)

# def callback_failsafe(data):
#    # Log incoming failsafe message
#    rospy.loginfo(rospy.get_caller_id()+"Failsafe = %s",data.data)    
#    sendpacket(data, 1)

# default failsafe settings (for when connection lost or button pressed)
fs_default = [0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5]

# try to get failsafe values from parameters
if not rospy.has_param('~failsafe'):
  rospy.logwarn('No failsafe parameter settings - using defaults')
  fs_data = fs_default
else:
  fs_data = rospy.get_param('~failsafe')
  if len(fs_data) == 8:
    rospy.loginfo('Using failsafe data: %s', fs_data)
  else:
    rospy.logwarn('Wrong number of channels (%d) in failsafe parameter - using defaults', len(fs_data))
    fs_data = fs_default

# and send the failsafe values to the box
sendpacket(fs_data, 1)

# Callbacks for incoming control messages
rospy.Subscriber("rcctrl", Twist, callback_ctrl)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

# if __name__ == '__main__':
#    rcbridge()
