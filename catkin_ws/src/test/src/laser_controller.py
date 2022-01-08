#!/usr/bin/env python

# laser_analyser.py created by Capt Tim Chisholm on 11 Jan 17 for RMC EE503

import roslib
import rospy
from lasertrackpkg.msg import laser_location_msg
from geometry_msgs.msg import Twist

def laser_controller():   
    rospy.init_node('laser_controller')
    rospy.Subscriber("/lasertrackpkg/laser_location", laser_location_msg, callback)   
    rospy.loginfo("laser_controller is now running...")  
    rospy.spin()
        
def callback(data):   
    if data.detected == 1:
        xalign = float(data.x)/float(data.xsize)
        rotatevel = 0.5-xalign
        pub = rospy.Publisher('/mobile_base/commands/velocity',Twist)    
        msg = Twist()
        msg.angular.z = rotatevel
        msg.linear.x=0.1   
        pub.publish(msg)
        rospy.loginfo('x: %i / %i relativeX: %f Rotation: %f' % (data.x, data.xsize, xalign, rotatevel))
  
if __name__ == '__main__':
    laser_controller()

    
    


    