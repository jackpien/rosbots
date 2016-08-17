#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist

ledPin = 23 # Broadcom pin 23 (P1 pin 16)
is_on = False

def callback(data):
    global is_on
    global ledPin
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.linear.x)
    if is_on:
        is_on = False
        GPIO.output(ledPin, GPIO.LOW)
    else:
        is_on = True
        GPIO.output(ledPin, GPIO.HIGH)
    
def motor_driver():
    global ledPin

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)

    rospy.Subscriber("twist", Twist, callback)

    GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
    GPIO.setup(ledPin, GPIO.OUT) # LED pin set as output 

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    GPIO.cleanup()

if __name__ == '__main__':
    motor_driver()
