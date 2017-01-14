#!/usr/bin/env python
import RPIO as GPIO #import RPi.GPIO as GPIO
from RPIO import PWM

import RPIO

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
pwm_subcycle_time_us = 20000 # 20ms cycle for PWM
pwm_max_width = 20000
pwm_granularity = 10

def shutdown_cb():
    global left_ia
    global left_ib
    global right_ia
    global right_ib
    global servo

    rospy.loginfo(rospy.get_caller_id() + " Shutdown callback!")
    
    GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

    servo.stop_servo(left_ia)
    servo.stop_servo(right_ia)

    GPIO.output(left_ib, GPIO.LOW)
    GPIO.output(right_ib, GPIO.LOW)
    GPIO.cleanup()

def callback(data):
    global left_ia
    global left_ib
    global right_ia
    global right_ib
    global is_on
    global servo
    global pwm_max_width
    global pwm_granularity
    
    rospy.loginfo(rospy.get_caller_id() + ": Linear.x: %f -- Angular.z: %f", data.linear.x, data.angular.z)
    x_dir = max(-1, min(1, data.linear.x))
    z_ang = max(-1, min(1, data.angular.z))

    lw = x_dir
    rw = x_dir

    if z_ang != 0:
        # Left wheel faster than right
        lw -= z_ang
        rw += z_ang

    lw = max(-1, min(1, lw))
    rw = max(-1, min(1, rw))

    rospy.loginfo(rospy.get_caller_id() + ": lw: %f -- rw: %f", lw, rw)

    if lw == 0:
        servo.set_servo(left_ia, 0) 
        GPIO.output(left_ib, GPIO.LOW)
    else:
        if lw > 0:
            pw = pwm_max_width * lw
            GPIO.output(left_ib, GPIO.LOW)
        else:
            pw = pwm_max_width - (pwm_max_width * (lw*-1))
            GPIO.output(left_ib, GPIO.HIGH)
            
        pw = int(pw/pwm_granularity) * pwm_granularity
        servo.set_servo(left_ia, pw) 
        

    if rw == 0:
        servo.set_servo(right_ia, 0) 
        GPIO.output(right_ib, GPIO.LOW)
    else:
        if rw > 0:
            pw = pwm_max_width * rw
            GPIO.output(right_ib, GPIO.LOW)
        else:
            pw = pwm_max_width - (pwm_max_width * (rw*-1))
            GPIO.output(right_ib, GPIO.HIGH)
            
        pw = int(pw/pwm_granularity) * pwm_granularity
        servo.set_servo(right_ia, pw) 

def gpio_callback(gpio_id, val):
    rospy.loginfo("gpio %s: %s", gpio_id, val)
            

def motor_driver():
    global left_ia
    global left_ib
    global right_ia
    global right_ib
    global servo
    global pwm_subcycle_time_us

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)

    rospy.Subscriber("twist", Twist, callback)    

    rospy.on_shutdown(shutdown_cb)

    GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
    GPIO.cleanup()
    #GPIO.setup(left_ia, GPIO.OUT)
    GPIO.setup(left_ib, GPIO.OUT)
    #GPIO.setup(right_ia, GPIO.OUT)

    GPIO.setup(right_ib, GPIO.OUT)
    
    GPIO.setup(22, GPIO.IN) # Right
    GPIO.setup(17, GPIO.IN) # Left

    servo = PWM.Servo(subcycle_time_us=pwm_subcycle_time_us)
    servo.set_servo(left_ia, 0) 
    servo.set_servo(right_ia, 0) 

    GPIO.output(left_ib, GPIO.LOW)
    GPIO.output(right_ib, GPIO.LOW)

    

    # Two GPIO interrupt callbacks
    RPIO.add_interrupt_callback(22, gpio_callback, edge='rising',
                                debounce_timeout_ms=10,
                                pull_up_down=RPIO.PUD_DOWN,
                                threaded_callback=True)
    RPIO.add_interrupt_callback(17, gpio_callback, edge='rising',
                                debounce_timeout_ms=10,
                                pull_up_down=RPIO.PUD_DOWN,
                                threaded_callback=True)

    # Starts waiting for interrupts 
    RPIO.wait_for_interrupts(threaded=True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    if False:
        rospy.loginfo(rospy.get_caller_id() + " Shutting down!")
        
        GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
        
        servo.stop_servo(left_ia)
        servo.stop_servo(right_ia)
        
        GPIO.setup(left_ia, GPIO.OUT)
        GPIO.setup(right_ia, GPIO.OUT)
        GPIO.output(left_ia, GPIO.LOW)
        GPIO.output(right_ia, GPIO.LOW)
        GPIO.output(left_ib, GPIO.LOW)
        GPIO.output(right_ib, GPIO.LOW)
        GPIO.cleanup()


if __name__ == '__main__':
    motor_driver()
