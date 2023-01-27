#!/bin/env python3
####
# This class implements the main methods for controlling the Robotiq2f85 gripper
#
####
from robotiq.robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import rospy

class Robotiq2f85:
    
    def __init__(self):
        self.command_publisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        
        self.command_publisher(self.reset())
        rospy.sleep(0.5)
        self.command_publisher(self.activate())

    def _clip(self, value, low_threshold, high_threshold):
        if value < low_threshold:
            return low_threshold
        if value > high_threshold:
            return high_threshold
        return value

    def reset(self):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 0
        return command
    
    def activate(self):
        
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150
        return command

    def open(self, force: int, speed: int):
        
        force = self._clip(force, 0, 255)
        speed = self._clip(speed, 0, 255)
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = speed
        command.rFR  = force
        command.rPR = 0
        return command

    def close(self, force, speed):
        
        force = self._clip(force, 0, 255)
        speed = self._clip(speed, 0, 255)
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = speed
        command.rFR  = force
        command.rPR = 255
        return command

    def set_positin(self, position, force, speed):
        
        force = self._clip(force, 0, 255)
        speed = self._clip(speed, 0, 255)
        position = self._clip(position, 0, 255)
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = speed
        command.rFR  = force
        command.rPR = position
        return command