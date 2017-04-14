#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def order():
    pub1 = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=10)
    pub2 = rospy.Publisher('/robot2/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.init_node('order', anonymous=True)
    r = rospy.Rate(10)
    vel1 = Twist()
    vel1.linear.x =0.1 
    vel1.angular.z = 0.5
    vel2 = Twist()
    vel2.angular.z = 0.5
    while not rospy.is_shutdown():
        pub1.publish(vel1)
        pub2.publish(vel2)
        r.sleep()

if __name__ == '__main__':
    try:
        order()
    except rospy.ROSInterruptException: pass
