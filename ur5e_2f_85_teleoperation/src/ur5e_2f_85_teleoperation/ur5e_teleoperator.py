from geometry_msgs.msg import Twist
import rospy
from ur5e_2f_85_controller.robotiq2f_85 import Robotiq2f85

class UR5eTeleoperator:
    
    INACTIVE = 0
    ACTIVE = 1

    def __init__(self, controller_name="twist_controller", topic_name='/joy', scale=[1., 1.]):
        
        self._gripper = Robotiq2f85()
        self._command_publisher = rospy.Publisher(f"{controller_name}/command", Twist, queue_size=1)
        self._controller_name = controller_name
        self._topic_name = topic_name
        
        # set scale for velocity command
        self._linear_scale = scale[0]
        self._angular_scale = scale[1]

        self._previous_gripper_button_state = 0

        # Teleoperation state, if inactive commands are not send
        rospy.loginfo("---- Teleoperator is inactive ----")
        self._teleoperator_state = UR5eTeleoperator.INACTIVE
        self._previous_teleoperator_activation_button_state = 0

        self._l2_first_message = True
        self._r2_first_message = True

    def convert_message_to_command(self, topic_name="joy", msg=None, linear_scale=1.0, angular_scale=1.0):
        """Convert the msg received into gripper and tcp twist commands.

        Args:
            topic_name (str): 
            msg ():
            linear_scale (float): 
            angular_scale (float):  
        Returns:
            (Twist, char): Tuple containing the Twist command and the gripper command

        """
        command_msg = Twist()
        gripper_command = None
        if topic_name == "/jow_twist":
            # set linear velocity
            command_msg.linear.x = msg.linear.x * linear_scale
            command_msg.linear.y = msg.linear.y * linear_scale
            command_msg.linear.z = msg.linear.z * linear_scale
            # set angular velocity
            command_msg.angular.x = msg.angular.x * angular_scale
            command_msg.angular.y = msg.angular.y * angular_scale
            command_msg.angular.z = msg.angular.z * angular_scale
            
        elif topic_name == "/joy":
            rospy.logdebug("Creating twist command from /joy topic")
            # set linear velocity
            command_msg.linear.x = msg.axes[0] * linear_scale 
            command_msg.linear.y = -msg.axes[1] * linear_scale # value positive -> positive y
            # R2 is used to to move towards positive z (e.g., goes down)
            # L2 is used to go move towards negative z (e.g., goes up)
            if self._r2_first_message and msg.axes[5] == 0.0:
                r2_value = 1.0
            else:
                self._r2_first_message = False
                r2_value = msg.axes[5] if msg.axes[5] >= 0 else 0
            
            if self._l2_first_message and msg.axes[2] == 0.0:
                l2_value = 1.0
            else:
                self._l2_first_message = False
                l2_value = msg.axes[2] if msg.axes[2] >= 0 else 0

            command_msg.linear.z = (l2_value - r2_value) * linear_scale
            # set angular velocity
            command_msg.angular.x = msg.axes[4] * angular_scale
            command_msg.angular.y = msg.axes[3] * angular_scale
            if msg.buttons[4] == 1:
                # counter-clockwise z rotation
                command_msg.angular.z = msg.buttons[4] * (angular_scale*2)
            elif msg.buttons[5] == 1:
                # clockwise z rotation
                command_msg.angular.z = (- msg.buttons[5]) * (angular_scale*2)
            
        
        # get gripper state
        gripper_state = self._gripper.get_state()
        # press square to close the gripper if the gripper is open
        if msg.buttons[3] == 1 and self._previous_gripper_button_state == 0 and \
           gripper_state['state'] == Robotiq2f85.ACTIVATE and gripper_state['gripper_open']:
            gripper_command = 'c'
            self._previous_gripper_button_state = 1
        # press square button to open the gripper if the gripper is closed
        elif msg.buttons[3] == 1 and self._previous_gripper_button_state == 0 and \
           gripper_state['state'] == Robotiq2f85.ACTIVATE and not gripper_state['gripper_open']:
            gripper_command = 'o'
            self._previous_gripper_button_state = 1
        elif msg.buttons[3] == 0 and self._previous_gripper_button_state == 1:
            self._previous_gripper_button_state = 0

        # press the cross to activate/inactive teleoperator
        if msg.buttons[0] == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == UR5eTeleoperator.INACTIVE:
            rospy.loginfo("---- Teleoperator is active ----")
            self._teleoperator_state = UR5eTeleoperator.ACTIVE
            self._previous_teleoperator_activation_button_state = 1
        elif msg.buttons[0] == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == UR5eTeleoperator.ACTIVE:
            rospy.loginfo("---- Teleoperator is inactive ----")
            self._teleoperator_state = UR5eTeleoperator.INACTIVE
            self._previous_teleoperator_activation_button_state = 1
        elif msg.buttons[0] == 0 and self._previous_teleoperator_activation_button_state == 1:
            self._previous_teleoperator_activation_button_state = 0

        return command_msg, gripper_command        


    def get_teleoperator_state(self):
        return self._teleoperator_state

    def send_command(self, msg):
        tcp_command, gripper_command = self.convert_message_to_command(topic_name=self._topic_name, msg=msg, linear_scale=self._linear_scale, angular_scale=self._angular_scale)
        if self._teleoperator_state == UR5eTeleoperator.ACTIVE:
            self._command_publisher.publish(tcp_command)
            self._gripper.send_command(command=gripper_command, position=-1, force=255, speed=255)