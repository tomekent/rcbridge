#!/usr/bin/env python

import numpy
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

#from std_msgs.msg import String
#from std_msgs.msg import MultiArrayDimension
#from std_msgs.msg import MultiArrayLayout
#from std_msgs.msg import UInt16MultiArray

#dim = MultiArrayDimension()
#dim.size = 8

#layout = MultiArrayLayout()
#layout.dim = dim
#layout.data_offset = 0

def ctrl():
    pub = rospy.Publisher('ctrl', numpy_msg(Floats)) #, queue_size=10)
    pub_fs = rospy.Publisher('failsafe', numpy_msg(Floats))
    rospy.init_node('ctrl', anonymous=True)
    r = rospy.Rate(10) # 10hz
    
    # Create failsafe message
    failsafe = numpy.array([0.5,0.5,0,0.5,0.5,0.5,0.5,0.5], dtype=numpy.float32)
    
    # Log failsafe message
    rospy.loginfo(failsafe)
    
    # Publish failsafe message
    pub_fs.publish(failsafe)
        
    while not rospy.is_shutdown():
        # Create control message
        ctrl = numpy.array([0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8], dtype=numpy.float32)
        
        # Log control message
        rospy.loginfo(ctrl)
        
        # Publish control message
        pub.publish(ctrl)
        
        # Handle timing
        r.sleep()
        
if __name__ == '__main__':
    try:
        ctrl()
    except rospy.ROSInterruptException: pass
