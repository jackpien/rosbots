#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.linear.x)
    
def motor_driver():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)

    rospy.Subscriber("twist", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    motor_driver()
