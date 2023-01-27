from geometry_msgs.msg import Twist
import rospy
from ur5e_2f_85_controller.robotiq2f_85 import Robotiq2f85

class UR5eTeleoperator:
    
    def __init__(self, controller_name="twist_controller", topic_name='/joy', scale=1):
        
        self._gripper = Robotiq2f85()
        self._command_publisher = rospy.Publisher(f"{controller_name}/command", Twist, queue_size=1)
        self._controller_name = controller_name
        self._topic_name = topic_name
        self._scale = scale
        self._previous_gripper_button_state = 0

    def convert_message_to_command(self, topic_name="joy", msg=None, scale=1.0):
        """Convert the msg received into gripper and tcp twist commands.

        Args:
            topic_name (str): 
            msg ():
            scale (float): 

        Returns:
            (Twist, char): Tuple containing the Twist command and the gripper command

        """
        command_msg = Twist()
        gripper_command = None
        if topic_name == "/jow_twist":
            # set linear velocity
            command_msg.linear.x = msg.linear.x * scale
            command_msg.linear.y = msg.linear.y * scale
            command_msg.linear.z = msg.linear.z * scale
            # set angular velocity
            command_msg.angular.x = msg.angular.x * scale
            command_msg.angular.y = msg.angular.y * scale
            command_msg.angular.z = msg.angular.z * scale
            
        elif topic_name == "/joy":
            rospy.logdebug("Creating twist command from /joy topic")
            # set linear velocity
            command_msg.linear.x = msg.axes[0] * scale 
            command_msg.linear.y = msg.axes[1] * scale # value positive -> positive y
            # L2 is used to to move towards positive z
            # R2 is used to go move towards negative z
            r2_value = msg.axes[5] if msg.axes[5] >= 0 else 0
            l2_value = msg.axes[2] if msg.axes[2] >= 0 else 0
        
            command_msg.linear.z = (l2_value - r2_value) * scale
            # set angular velocity
            command_msg.angular.x = msg.axes[3] * scale
            command_msg.angular.y = msg.axes[4] * scale
            if msg.buttons[4] == 1:
                # counter-clockwise z rotation
                command_msg.angular.z = msg.buttons[4] * scale
            elif msg.buttons[5] == 1:
                # clockwise z rotation
                command_msg.angular.z = (- msg.buttons[5]) * scale
            
        
        # press square to close the gripper if the gripper is open
        if msg.buttons[3] == 1 and self._previous_gripper_button_state == 0 and self._gripper.get_state() == Robotiq2f85.OPEN:
            gripper_command = 'c'
            self._previous_gripper_button_state = 1
        # press square button to open the gripper if the gripper is closed
        elif msg.buttons[3] == 1 and self._previous_gripper_button_state == 0 and self._gripper.get_state() == Robotiq2f85.CLOSE:
            gripper_command = 'o'
            self._previous_gripper_button_state = 1
        elif msg.buttons[3] == 0 and self._previous_gripper_button_state == 1:
            self._previous_gripper_button_state = 0

        return command_msg, gripper_command        


    def send_command(self, msg):
        tcp_command, gripper_command = self.convert_message_to_command(topic_name=self._topic_name, msg=msg, scale=self._scale)
        self._command_publisher.publish(tcp_command)
        self._gripper.send_command(command=gripper_command, position=-1, force=255, speed=255)