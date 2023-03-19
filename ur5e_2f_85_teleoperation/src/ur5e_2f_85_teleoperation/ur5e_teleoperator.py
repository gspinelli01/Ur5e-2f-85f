from geometry_msgs.msg import Twist
import rospy
from ur5e_2f_85_controller.robotiq2f_85 import Robotiq2f85
from ur5e_2f_85_teleoperation.msg import TrajectoryState
from ds4_driver.msg import Feedback

class UR5eTeleoperator:
    
    INACTIVE = 0
    ACTIVE = 1

    def __init__(self, controller_name="twist_controller", topic_name='/joy', scale=[1., 1.]):
        
        self._gripper = Robotiq2f85()
        self._command_publisher = rospy.Publisher(f"{controller_name}/command", Twist, queue_size=1)
        
        self._trajectory_state_publisher = rospy.Publisher("trajectory_state", TrajectoryState, queue_size=1)
        self._trajectory_state_msg = TrajectoryState()
        self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_IDLE
        self._previous_trajectory_state_button_state = 0
        self._previous_trj_state = TrajectoryState.TRAJECTORY_IDLE
        self._rumble_counter = 0

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

        # Publisher for joy feedback
        self._joy_feedback_msg = Feedback()
        self._joy_feedback_msg.set_led = True
        self._joy_feedback_msg.set_rumble = True
        self._joy_feedback_msg.rumble_duration = 0.2
        self._joy_feedback_publisher = rospy.Publisher("/set_feedback", Feedback, queue_size=1)

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
            # if self._r2_first_message and msg.axes[5] == 0.0:
            #     r2_value = 1.0
            # else:
            #     self._r2_first_message = False
            #     r2_value = msg.axes[5] if msg.axes[5] >= 0 else 0
            
            # if self._l2_first_message and msg.axes[2] == 0.0:
            #     l2_value = 1.0
            # else:
            #     self._l2_first_message = False
            #     l2_value = msg.axes[2] if msg.axes[2] >= 0 else 0
            l2_value = msg.axes[4]
            r2_value = msg.axes[5]
            if r2_value != 0.0 and l2_value == 0.0:
                command_msg.linear.z = r2_value * linear_scale
            elif r2_value == 0.0 and l2_value != 0.0:
                command_msg.linear.z = -l2_value * linear_scale
            
            # set angular velocity
            command_msg.angular.x = msg.axes[3] * angular_scale
            command_msg.angular.y = msg.axes[2] * angular_scale
            if msg.buttons[4] == 1:
                # counter-clockwise z rotation
                command_msg.angular.z = msg.buttons[4] * (angular_scale*2)
            elif msg.buttons[6] == 1:
                # clockwise z rotation
                command_msg.angular.z = (- msg.buttons[5]) * (angular_scale*2)
            
            # get gripper state
            # gripper_state = self._gripper.get_state()
            # # press square to close the gripper if the gripper is open
            # if msg.buttons[0] == 1 and self._previous_gripper_button_state == 0 and \
            #    gripper_state['state'] == Robotiq2f85.ACTIVATE and gripper_state['gripper_open']:
            #     gripper_command = 'c'
            #     self._previous_gripper_button_state = 1
            # # press square button to open the gripper if the gripper is closed
            # elif msg.buttons[0] == 1 and self._previous_gripper_button_state == 0 and \
            #    gripper_state['state'] == Robotiq2f85.ACTIVATE and not gripper_state['gripper_open']:
            #     gripper_command = 'o'
            #     self._previous_gripper_button_state = 1
            # elif msg.buttons[0] == 0 and self._previous_gripper_button_state == 1:
            #     self._previous_gripper_button_state = 0

            # press the cross to activate/inactive teleoperator
            if msg.buttons[3] == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == UR5eTeleoperator.INACTIVE:
                rospy.loginfo("---- Teleoperator is active ----")
                self._teleoperator_state = UR5eTeleoperator.ACTIVE
                self._previous_teleoperator_activation_button_state = 1
            elif msg.buttons[3] == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == UR5eTeleoperator.ACTIVE:
                rospy.loginfo("---- Teleoperator is inactive ----")
                self._teleoperator_state = UR5eTeleoperator.INACTIVE
                self._previous_teleoperator_activation_button_state = 1
            elif msg.buttons[3] == 0 and self._previous_teleoperator_activation_button_state == 1:
                self._previous_teleoperator_activation_button_state = 0

            # get trajectory state
            # press circle button
            if msg.buttons[2] == 1 and self._previous_trajectory_state_button_state == 0:
                self._previous_trajectory_state_button_state = 1
                if self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_IDLE:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_START
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_START:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_APPROACHING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_APPROACHING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_PICKING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_PICKING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_MOVING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_MOVING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_PLACING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_PLACING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_END
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_IDLE
            elif msg.buttons[2] == 0 and self._previous_trajectory_state_button_state == 1:
                self._previous_trajectory_state_button_state = 0

        elif topic_name == "/status":
            rospy.logdebug("Creating twist command from /joy topic")
            # set linear velocity
            command_msg.linear.x = msg.axis_left_x * linear_scale 
            command_msg.linear.y = -msg.axis_left_y * linear_scale # value positive -> positive y
            # R2 is used to to move towards positive z (e.g., goes down)
            # L2 is used to go move towards negative z (e.g., goes up)
            l2_value = msg.axis_l2
            r2_value = msg.axis_r2
            if r2_value != 0.0 and l2_value == 0.0:
                command_msg.linear.z = r2_value * linear_scale
            elif r2_value == 0.0 and l2_value != 0.0:
                command_msg.linear.z = -l2_value * linear_scale
            
            # set angular velocity
            command_msg.angular.x = msg.axis_right_y * angular_scale
            command_msg.angular.y = -msg.axis_right_x * angular_scale
            if msg.button_r1 == 1:
                # clockwise z rotation
                command_msg.angular.z = -msg.button_r1 * (angular_scale*2)
            elif msg.button_l1 == 1:
                # counter-clockwise z rotation
                command_msg.angular.z = (msg.button_l1) * (angular_scale*2)

    
            # get gripper state
            gripper_state = self._gripper.get_state()
            # press square to close the gripper if the gripper is open
            if msg.button_square == 1 and self._previous_gripper_button_state == 0 and \
               gripper_state['state'] == Robotiq2f85.ACTIVATE and gripper_state['gripper_open']:
                gripper_command = 'c'
                self._previous_gripper_button_state = 1
            # press square button to open the gripper if the gripper is closed
            elif msg.button_square == 1 and self._previous_gripper_button_state == 0 and \
               gripper_state['state'] == Robotiq2f85.ACTIVATE and not gripper_state['gripper_open']:
                gripper_command = 'o'
                self._previous_gripper_button_state = 1
            elif msg.button_square == 0 and self._previous_gripper_button_state == 1:
                self._previous_gripper_button_state = 0

            # press the cross to activate/inactive teleoperator
            if msg.button_cross == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == UR5eTeleoperator.INACTIVE:
                rospy.loginfo("---- Teleoperator is active ----")
                self._teleoperator_state = UR5eTeleoperator.ACTIVE
                self._previous_teleoperator_activation_button_state = 1
            elif msg.button_cross == 1 and self._previous_teleoperator_activation_button_state == 0 and self._teleoperator_state == UR5eTeleoperator.ACTIVE:
                rospy.loginfo("---- Teleoperator is inactive ----")
                self._teleoperator_state = UR5eTeleoperator.INACTIVE
                self._previous_teleoperator_activation_button_state = 1
            elif msg.button_cross == 0 and self._previous_teleoperator_activation_button_state == 1:
                self._previous_teleoperator_activation_button_state = 0

            # get trajectory state
            # press circle button
            if msg.button_circle == 1 and self._previous_trajectory_state_button_state == 0:
                self._previous_trajectory_state_button_state = 1
                if self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_IDLE:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_START
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_START:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_APPROACHING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_APPROACHING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_PICKING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_PICKING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_MOVING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_MOVING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_PLACING
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_PLACING:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_END
                elif self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_END:
                    self._trajectory_state_msg.trajectory_state = TrajectoryState.TRAJECTORY_IDLE
            elif msg.button_circle == 0 and self._previous_trajectory_state_button_state == 1:
                self._previous_trajectory_state_button_state = 0

        return command_msg, gripper_command        


    def get_teleoperator_state(self):
        return self._teleoperator_state

    def send_command(self, msg):
        tcp_command, gripper_command = self.convert_message_to_command(topic_name=self._topic_name, msg=msg, linear_scale=self._linear_scale, angular_scale=self._angular_scale)
        if self._teleoperator_state == UR5eTeleoperator.ACTIVE:
            # set the led to blue if the teleoperator is active and the trajectory state is idle
            if self._trajectory_state_msg.trajectory_state == TrajectoryState.TRAJECTORY_IDLE:
                self._joy_feedback_msg.led_b = 1.0
                self._joy_feedback_msg.led_g = 0.0
                self._joy_feedback_msg.led_r = 0.0
            # set the led to gree if the teleoperator is active and the trajectory state is not idle
            elif self._trajectory_state_msg.trajectory_state != TrajectoryState.TRAJECTORY_IDLE:
                self._joy_feedback_msg.led_b = 0.0
                self._joy_feedback_msg.led_g = 1.0
                self._joy_feedback_msg.led_r = 0.0

            self._command_publisher.publish(tcp_command)
            self._gripper.send_command(command=gripper_command, position=-1, force=255, speed=255)
            self._trajectory_state_publisher.publish(self._trajectory_state_msg)
            self._joy_feedback_publisher.publish(self._joy_feedback_msg)
        elif self._teleoperator_state == UR5eTeleoperator.INACTIVE:
            # Set the led to red if the teleoperator is inactive
            self._joy_feedback_msg.led_b = 0.0
            self._joy_feedback_msg.led_g = 0.0
            self._joy_feedback_msg.led_r = 1.0
            self._joy_feedback_publisher.publish(self._joy_feedback_msg)