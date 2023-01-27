#!/bin/env python3
####
# This class implements the main methods for controlling the Robotiq2f85 gripper
#
####
from robotiq_2f_gripper_control.baseRobotiq2FGripper import robotiqbaseRobotiq2FGripper
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import rospy

class Robotiq2f85:

    # list of states for gripper
    INACTIVE = 0
    OPEN = 1
    CLOSE = 2

    def __init__(self):

        self.command_publisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        
        self._state=Robotiq2f85.INACTIVE
        self.command_publisher.publish(self.reset())
        rospy.sleep(0.5)
        self.command_publisher.publish(self.activate())
        self._state=Robotiq2f85.OPEN
    
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

    def send_command(self, command='a', position=-1, force=255, speed=255):
        if command == 'a':
            self.command_publisher.publish(self.activate())
            self._state=Robotiq2f85.OPEN
        elif command == 'r':
            self.command_publisher.publish(self.reset())
        elif command == 'o':
            self.command_publisher.publish(self.open(force=force, speed=speed))
            self._state=Robotiq2f85.OPEN
        elif command == 'c':
            self.command_publisher.publish(self.close(force=force, speed=speed))
            self._state=Robotiq2f85.CLOSE
        elif command == 's':
            if position<0:
                rospy.logerr("For set_position command, position is required")
            else:
                self.command_publisher.publish(self.set_positin(position=position, force=force, speed=speed))

    def get_state(self):
        return self._state