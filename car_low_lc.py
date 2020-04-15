#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from sensor_msgs.msg import Joy
from pca9685 import PCA9685

class ActuatorNode:

    pca9685 = None
    const_throttle = 0
    #Steering constants
    MAX_LEFT_ANGLE = rospy.get_param("/MAX_LEFT_ANGLE")
    MIN_RIGHT_ANGLE = rospy.get_param("/MIN_RIGHT_ANGLE")
    MAX_STEERING_PULSE = rospy.get_param("/MAX_STEERING_PULSE")
    MIN_STEERING_PULSE = rospy.get_param("/MIN_STEERING_PULSE")
    START_RECORDING  = rospy.get_param("/START_RECORDING")
    STEERING_CHANNEL = rospy.get_param("/STEERING_CHANNEL")

    #Throttle constants
    MIN_THROTTLE = rospy.get_param("/MIN_THROTTLE")
    MAX_THROTTLE = rospy.get_param("/MAX_THROTTLE")
    ZERO_PULSE = rospy.get_param("/ZERO_PULSE")
    MIN_THROTTLE_PULSE = rospy.get_param("/MIN_THROTTLE_PULSE")
    MAX_THROTTLE_PULSE = rospy.get_param("/MAX_THROTTLE_PULSE")
    THROTTLE_CHANNEL = rospy.get_param("/THROTTLE_CHANNEL")
    MAX_REVERSE_THROTTLE = rospy.get_param("/MAX_REVERSE_THROTTLE")
    CONST_THROTTLE_STEP = rospy.get_param("/CONST_THROTTLE_STEP")
    
    AUTO_DRIVE = None

    def __init__(self):
        self.pca9685 = PCA9685()
        self.const_throttle = rospy.get_param("/CONST_THROTTLE")

    def autodrive(self, data):
        commands = data.data
        command = commands.split(":")
        rospy.loginfo('Autodrive is on')
        steering_pulse = self.getActuatorPulseValue(float(command[0]), self.MIN_RIGHT_ANGLE, self.MAX_LEFT_ANGLE, self.MIN_STEERING_PULSE, self.MAX_STEERING_PULSE, channel_type='steering')
        throttle_pulse = self.getActuatorPulseValue(float(command[1]), self.MIN_THROTTLE, self.MAX_THROTTLE, self.MIN_THROTTLE_PULSE, self.MAX_THROTTLE_PULSE, channel_type='throttle')
        self.pca9685.set_pwm_value(self.STEERING_CHANNEL, pulse=steering_pulse)
        self.pca9685.set_pwm_value(self.THROTTLE_CHANNEL, pulse=throttle_pulse)
        rospy.loginfo('Steering value: %s', steering_pulse)
        rospy.loginfo('Throttle value: %s', throttle_pulse)


    

    def getActuatorPulseValue(self, joystickValue, joystickMin, joystickMax, actuatorMin, actuatorMax, channel_type='steering'):
        
        #Linear mapping between two ranges of values
        joystickRange = joystickMax - joystickMin
        actuatorRange = actuatorMax - actuatorMin

        joystickActuatorRatio = float(joystickRange)/float(actuatorRange)
        actuatorValue = int((joystickValue - joystickMin) / joystickActuatorRatio + actuatorMin)	   
        if channel_type == 'steering':
		   actuatorValue +=18
        
        return actuatorValue



def listener(actuatorNode):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('actuator_node', anonymous=True)
    rospy.Subscriber('autodrive', String, actuatorNode.autodrive, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    actuatorNode = ActuatorNode()
    listener(actuatorNode)