#!/bin/env python3
####
# This class implements the main methods for controlling the Robotiq2f85 gripper
#
####
from robotiq_2f_gripper_control.baseRobotiq2FGripper import robotiqbaseRobotiq2FGripper
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
import rospy

class Robotiq2f85:
    
    # used for initialization, when values are not defined 
    NOT_DEFINED = -1

    # list of gripper state
    RESET = 0
    ACTIVATE = 1

    # list of error messages
    NO_FAULT_BIT = 1 
    FAULT_BIT = 0
    NO_FAULT = "No fault"
    ACTION_DELAYES = "Action delayed, activation (reactivation) must be completed prior to perfmoring the action"
    NEED_OF_ACTIVATION = "The activation bit must be set prior to action"
    MAX_TEMP = "Maximum operating temperature exceeded, wait for cool-down"
    COMMUNICATION_ERROR = "No communication during at least 1 second"
    MINIMUM_OPERATING_VOLTAGE = "Under minimum operating voltage"
    AUTOMATIC_RELEASE_PROGRESS = "Automatic release in progress"
    INTERNAL_FAULT = "Internal fault; contact support@robotiq.com"
    ACTIVATION_FAULT = "Activation fault, verify that no interference or other error occurred" 
    OVERCURRENT = "Overcurrent triggered"
    AUTOMATIC_RELEASE_COMPLETED= "Automatic release completed" 

    # Moving state
    STAND_BY = 0
    STAND_BY_MSG = 'Standby (or performing activation/automatic release)'
    GO_TO = 1
    GO_TO_MSG = 'Go to Position Request'

    # Activation state
    ACTIVATION_RESET = 0
    ACTIVATION_RESET_MSG = 'Gripper is in reset ( or automatic release ) state. see Fault Status if Gripper is activated'
    ACTIVATION_IN_PROGRESS = 1
    ACTIVATION_IN_PROGRESS_MSG = 'Activation in progress'
    ACTIVATION_COMPLETED = 3
    ACTIVATION_COMPLETED_MSG = 'Activation is completed'

    # Object detection state
    MOVING_FINGERS = 0
    MOVING_FINGERS_MSG = 'Fingers are in motion (only meaningful if gGTO = 1)'
    STOPPED_WHILE_OPENING = 1
    STOPPED_WHILE_OPENING_MSG = 'Fingers have stopped due to a contact while opening'
    STOPPED_WHILE_CLOSING = 2
    STOPPED_WHILE_CLOSING_MSG =  'Fingers have stopped due to a contact while closing'
    STOPPED_AT_REQUESTED_POSITION = 3
    STOPPED_AT_REQUESTED_POSITION_MSG = 'Fingers are at requested position'

    def __init__(self):

        self.command_publisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        
        self._state = {
            # Gripper state
            # Can be reset or active
            'state': Robotiq2f85.NOT_DEFINED,
            # activation state
            'activation_state': Robotiq2f85.NOT_DEFINED,
            'activation_state_msg': Robotiq2f85.NOT_DEFINED,
            # Gripper fault state and message
            # _fault_state set to 1 when fault message is different from NO_FAULT
            'fault_state': Robotiq2f85.NOT_DEFINED,
            'fault_msg': Robotiq2f85.NOT_DEFINED,
            # Moving state
            'go_to_state': Robotiq2f85.NOT_DEFINED,
            'go_to_msg':Robotiq2f85.NOT_DEFINED,
            # Requested position echo
            'requested_position_echo': Robotiq2f85.NOT_DEFINED,
            # Current position
            'finger_position': Robotiq2f85.NOT_DEFINED,
            # Finger current
            'finger_current': Robotiq2f85.NOT_DEFINED,
            # Object detection
            'obj_detection': Robotiq2f85.NOT_DEFINED,
            'obj_detection_msg': Robotiq2f85.NOT_DEFINED,
            # Finger state
            # True if the gripper is full open, False if the gripper is closed (full closed, closed at requested position, or closed due to gripper object)
            'gripper_open': Robotiq2f85.NOT_DEFINED,
        }

        rospy.loginfo("---- Reset gripper ----")
        while self._state['state'] != Robotiq2f85.RESET:
            self.send_command(command='r')
            rospy.sleep(0.5)
            if self._state['state'] != Robotiq2f85.RESET:
                rospy.logerr("ERROR DURING RESET")
                rospy.logerr(f"Fault message: {self._state['fault_msg']}")

        rospy.loginfo("---- Activate gripper ----")
        self.send_command(command='a')
        # wait until activation is completed
        while self._state['activation_state'] != Robotiq2f85.ACTIVATION_COMPLETED:
            rospy.loginfo("Waiting for gripper activation completion....")
            if self._state['activation_state'] == Robotiq2f85.ACTIVATION_RESET:
                rospy.logerr(f"{self._state['fault_msg']}") 
            rospy.sleep(0.5)
            self.update_state()
        rospy.loginfo("Activation completed, the gripper is ready")

        
    def _clip(self, value, low_threshold, high_threshold):
        if value < low_threshold:
            return low_threshold
        if value > high_threshold:
            return high_threshold
        return value

    def _state_interpreter(self, state_msg):
        """Read the bytes stored in the state_msg and set the corresponding variable state
        """

        # gACT
        # check if the gripper is active or not 
        if(state_msg.gACT == 0):
            self._state['state'] = Robotiq2f85.RESET
        elif(state_msg.gACT == 1):
            self._state['state'] = Robotiq2f85.ACTIVATE

        # gFLT
        # check for fault messages
        if(state_msg.gFLT == 0x00):
            self._state['fault_state'] = Robotiq2f85.NO_FAULT_BIT
            self._state['fault_msg'] = Robotiq2f85.NO_FAULT
        if(state_msg.gFLT == 0x05):
            self._state['fault_state'] = Robotiq2f85.FAULT_BIT
            self._state['fault_msg'] = Robotiq2f85.ACTION_DELAYES
        if(state_msg.gFLT == 0x07):
            self._state['fault_state'] = Robotiq2f85.FAULT_BIT
            self._state['fault_msg'] = Robotiq2f85.ACTIVATION_FAULT
        if(state_msg.gFLT == 0x09):
            self._state['fault_state'] = Robotiq2f85.FAULT_BIT
            self._state['fault_msg'] = Robotiq2f85.COMMUNICATION_ERROR
        if(state_msg.gFLT == 0x0B):
            self._state['fault_state'] = Robotiq2f85.FAULT_BIT
            self._state['fault_msg'] = Robotiq2f85.AUTOMATIC_RELEASE_PROGRESS
        if(state_msg.gFLT == 0x0E):
            self._state['fault_state'] = Robotiq2f85.FAULT_BIT
            self._state['fault_msg'] = Robotiq2f85.OVERCURRENT
        if(state_msg.gFLT == 0x0F):
            self._state['fault_state'] = Robotiq2f85.FAULT_BIT
            self._state['fault_msg'] = Robotiq2f85.AUTOMATIC_RELEASE_COMPLETED

        # gSTA
        # check for activation state
        if(state_msg.gSTA == 0):
            self._state['activation_state'] = Robotiq2f85.ACTIVATION_RESET
            self._state['activation_state_msg'] =  Robotiq2f85.ACTIVATION_RESET_MSG
        if(state_msg.gSTA == 1):
            self._state['activation_state'] = Robotiq2f85.ACTIVATION_IN_PROGRESS
            self._state['activation_state_msg'] =  Robotiq2f85.ACTIVATION_IN_PROGRESS_MSG
        if(state_msg.gSTA == 3):
            self._state['activation_state'] = Robotiq2f85.ACTIVATION_COMPLETED
            self._state['activation_state_msg'] =  Robotiq2f85.ACTIVATION_COMPLETED_MSG

        # gGTO
        # check for moving state
        if(state_msg.gGTO == 0):
            self._state['go_to_state'] = Robotiq2f85.STAND_BY
            self._state['go_to_msg'] = Robotiq2f85.STAND_BY_MSG
        if(state_msg.gGTO == 1):
            self._state['go_to_state'] = Robotiq2f85.GO_TO
            self._state['go_to_msg'] = Robotiq2f85.GO_TO_MSG

        # gPR
        # requested position, to obtain position in mm
        self._state['requested_position_echo'] = int(state_msg.gPR)

        # gPO
        # check for current finger position
        self._state['finger_position'] = int(state_msg.gPO)

        # gCU
        # check for fingers current, 10 * value read [mA]
        self._state['finger_current'] = state_msg.gCU

        # gOBJ
        # check for object state
        if(state_msg.gOBJ == 0):
            self._state['obj_detection'] = Robotiq2f85.MOVING_FINGERS
            self._state['obj_detection_msg'] = Robotiq2f85.MOVING_FINGERS_MSG
        if(state_msg.gOBJ == 1):
            self._state['obj_detection'] = Robotiq2f85.STOPPED_WHILE_OPENING
            self._state['obj_detection_msg'] = Robotiq2f85.STOPPED_WHILE_OPENING_MSG
        if(state_msg.gOBJ == 2):
            self._state['obj_detection'] = Robotiq2f85.STOPPED_WHILE_CLOSING
            self._state['obj_detection_msg'] = Robotiq2f85.STOPPED_WHILE_CLOSING_MSG
        if(state_msg.gOBJ == 3):
            self._state['obj_detection'] = Robotiq2f85.STOPPED_AT_REQUESTED_POSITION
            self._state['obj_detection_msg'] = Robotiq2f85.STOPPED_AT_REQUESTED_POSITION_MSG

        if self._state['finger_position'] >= 0 and self._state['finger_position'] <= 5:
            # gripper is full open
            self._state['gripper_open'] = True
        if self._state['finger_position'] >= 220:
            # gripper is full closed
            self._state['gripper_open'] = False    
        if self._state['obj_detection'] == Robotiq2f85.STOPPED_WHILE_CLOSING:
            # gripper is closed due to object picking
            self._state['gripper_open'] = False
        if self._state['obj_detection'] == Robotiq2f85.STOPPED_WHILE_OPENING:
            # gripper is open
            self._state['gripper_open'] = True


    def update_state(self):
        state_msg = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
        self._state_interpreter(state_msg=state_msg)
    
    def reset(self):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 0
        rospy.logdebug(f"Resetting the gripper with command \n{command}")
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
        # 1. Send command
        if command == 'a':
            self.command_publisher.publish(self.activate())
        
        elif command == 'r':
            rospy.logdebug("Resetting the gripper")
            self.command_publisher.publish(self.reset())

        elif command == 'o':
            rospy.loginfo("Sending open command to gripper")
            self.command_publisher.publish(self.open(force=force, speed=speed))

        elif command == 'c':
            rospy.loginfo("Sending close command to gripper")
            self.command_publisher.publish(self.close(force=force, speed=speed))

        elif command == 's':
            if position<0:
                rospy.logerr("For set_position command, position is required")
            else:
                self.command_publisher.publish(self.set_positin(position=position, force=force, speed=speed))

        # 2. Update the internal state variable
        self.update_state()

    def get_state(self):
        self.update_state()
        return self._state

        