#!/usr/bin/env python
import RPi.GPIO as GPIO
from RPIO import PWM

import rospy
from geometry_msgs.msg import Twist

#ledPin = 23 # Broadcom pin 23 (P1 pin 16)
# Broadcom pin outs
# https://www.element14.com/community/servlet/JiveServlet/previewBody/73950-102-10-339300/pi3_gpio.png
left_ia = 23
left_ib = 24
right_ia = 20
right_ib = 21

is_on = False
servo = None

def shutdown_cb():
    servo.stop_servo(left_ia)
    servo.stop_servo(right_ia)
    GPIO.cleanup()

def callback(data):
    global is_on
    global servo
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.linear.x)
    if data.linear.x > 0:
        pw = (3 + (17.0 * data.linear.x)) * 1000.0
        servo.set_servo(left_ia, pw) 
        servo.set_servo(right_ia, pw)
        #GPIO.output(left_ia, GPIO.HIGH)
        #GPIO.output(right_ia, GPIO.HIGH)
    else:
        servo.set_servo(left_ia, 0) 
        servo.set_servo(right_ia, 0) 
        #GPIO.output(left_ia, GPIO.LOW)
        #GPIO.output(right_ia, GPIO.LOW)

def motor_driver():
    global left_ia
    global left_ib
    global right_ia
    global right_ib
    global servo

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)

    rospy.Subscriber("twist", Twist, callback)    

    rospy.on_shutdown(shutdown_cb)

    GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
    #GPIO.setup(left_ia, GPIO.OUT)
    GPIO.setup(left_ib, GPIO.OUT)
    #GPIO.setup(right_ia, GPIO.OUT)
    GPIO.setup(right_ib, GPIO.OUT)

    servo = PWM.Servo()
    servo.set_servo(left_ia, 0) 
    servo.set_servo(right_ia, 0) 

    GPIO.output(left_ib, GPIO.LOW)
    GPIO.output(right_ib, GPIO.LOW)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    motor_driver()
