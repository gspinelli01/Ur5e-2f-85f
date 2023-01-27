from geometry_msgs.msg import Twist
import rospy

class UR5e_teleoperator:
    
    def __init__(self):
        self.gripper = None


    def convert_message_to_command(topic_name="joy", msg=None, scale=1.0):
        command_msg = Twist()
        if topic_name == "/jow_twist":
            # set linear velocity
            command_msg.linear.x = msg.linear.x * scale
            command_msg.linear.y = msg.linear.y * scale
            command_msg.linear.z = msg.linear.z * scale
            # set angular velocity
            command_msg.angular.x = msg.angular.x * scale
            command_msg.angular.y = msg.angular.y * scale
            command_msg.angular.z = msg.angular.z * scale
            return command_msg
        elif topic_name == "/joy":
            rospy.logdebug("Creating twist command from /joy topic")
            # set linear velocity
            command_msg.linear.x = -msg.axes[0] * scale
            command_msg.linear.y = msg.axes[1] * scale
            # RT is used to to move towards positive z
            # LT is used to go move towards negative z
            rt_value = msg.axes[5] if msg.axes[5] >= 0 else 0
            lt_value = msg.axes[2] if msg.axes[2] >= 0 else 0
        
            command_msg.linear.z = (lt_value - rt_value) * scale
            # set angular velocity
            command_msg.angular.x = msg.axes[3] * scale
            command_msg.angular.y = msg.axes[4] * scale
            if msg.buttons[4] == 1:
                # counter-clockwise z rotation
                command_msg.angular.z = msg.buttons[4] * scale
            elif msg.buttons[5] == 1:
                # clockwise z rotation
                command_msg.angular.z = (- msg.buttons[5]) * scale
            return command_msg
        
        # press x to close the