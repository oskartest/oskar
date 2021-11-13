
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from RPi import GPIO
from gpiozero import Motor
from sensor_msgs.msg import LaserScan


pin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
motor = Motor(forward=20, backward=21)
p = GPIO.PWM(pin, 50)


p.start(0)
SERVO_MAX_DUTY = 12.5 
SERVO_MIN_DUTY = 2.5

steer = 0

sub = None

def setServoPos(degree):

    if degree > 180:
        degree = 180

    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    p.ChangeDutyCycle(duty)

def callback(data):
    steer = data.data
    rospy.loginfo(steer)
    angle = 90-steer*180
    setServoPos(angle)
    
    
    
def callback_lidar(laser):
    center = laser.ranges[89]
    left = laser.ranges[0]
    right = laser.ranges[179]
    around = laser.ranges[0:180]
    perpection = laser.ranges[45:135]
    if perpection >= 0.3 :
        motor.forward(speed=0.5)



    
    
    
    
def main():
    global sub
    rospy.init_node('sub_steer_lidar', anonymous=True)
    sub = rospy.Subscriber('/steers', Float32, callback)
    rospy.Subscriber('/scan', LaserScan, callback_lidar)
    rospy.spin()

    

if __name__ == '__main__':
    main()



