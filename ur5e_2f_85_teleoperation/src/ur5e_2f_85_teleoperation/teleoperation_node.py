#!/usr/bin/env python3

####
# This node: 
# (1) Start the twist_controller
# (2) Subscribe to the Twist topic where the controller's twist information are published
# (3) Publish the Twist message to /twist_controller/command topic
#
####
import rospy
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchControllerResponse, UnloadController, UnloadControllerRequest, LoadController, LoadControllerRequest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ur5e_teleoperator import UR5eTeleoperator

def main():

    def shutdown_operations():
        rospy.loginfo(f"Stop {controller_name}")
        controller_manager_switch_msg = SwitchControllerRequest()
        controller_manager_switch_msg.start_controllers = ['scaled_pos_joint_traj_controller']
        controller_manager_switch_msg.stop_controllers = [controller_name]
        controller_manager_switch_msg.strictness = 1
        controller_manager_switch_response = controller_manager_switch_srv(controller_manager_switch_msg)
        if controller_manager_switch_response.ok:
            rospy.loginfo(f"{controller_name} stopped")
        else:
            rospy.logerr(f"{controller_name} not stopped")

        rospy.loginfo(f"Unload {controller_name}")
        rospy.wait_for_service("controller_manager/unload_controller")
        controller_manager_unload_srv = rospy.ServiceProxy("controller_manager/unload_controller", UnloadController)
        controller_manager_unload_msg = UnloadControllerRequest()
        controller_manager_unload_msg.name = controller_name
        controller_manager_unload_response = controller_manager_unload_srv(controller_manager_unload_msg)
        if controller_manager_unload_response.ok:
            rospy.loginfo(f"{controller_name} unloaded correctly")
        else:
            rospy.logerr(f"{controller_name} not unloaded")

    rospy.init_node("teleoperation_node")
    
    rospy.on_shutdown(shutdown_operations)
    
    rate = rospy.Rate(50)
    # get parameters 
    scale = rospy.get_param("/scale")
    controller_name = rospy.get_param("/controller_name")
    topic_name = rospy.get_param("/topic_name")

    # load controller
    rospy.wait_for_service("controller_manager/load_controller")
    controller_manager_load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
    controller_manager_load_msg = LoadControllerRequest()
    controller_manager_load_msg.name = controller_name
    controller_manager_load_response = controller_manager_load_srv(controller_manager_load_msg)
    if controller_manager_load_response.ok:
        rospy.loginfo(f"{controller_name} loaded correctly")
    else:
        rospy.logerr(f"{controller_name} not loaded")

    # start controller
    rospy.sleep(1)
    rospy.wait_for_service("controller_manager/switch_controller")
    controller_manager_switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
    controller_manager_switch_msg = SwitchControllerRequest()
    controller_manager_switch_msg.start_controllers = [controller_name]
    controller_manager_switch_msg.stop_controllers = ['scaled_pos_joint_traj_controller']
    controller_manager_switch_msg.strictness = 1
    controller_manager_switch_response = controller_manager_switch_srv(controller_manager_switch_msg)
    if controller_manager_switch_response.ok:
        rospy.loginfo(f"{controller_name} started correctly")
    else:
        rospy.logerr(f"{controller_name} not started")

    # create command publisher
    ur_teleoperator = UR5eTeleoperator(controller_name=controller_name, topic_name=topic_name, scale=scale)
    rospy.loginfo(f"---- Starting to forward the {topic_name} message, with scale {scale} ----")
    msg = None
    while not rospy.is_shutdown():
        if topic_name == "/joy_twist":
            msg = rospy.wait_for_message(topic_name, Twist)
        elif topic_name == "/joy":
            msg = rospy.wait_for_message(topic_name, Joy)

        rospy.logdebug(msg)
        ur_teleoperator.send_command(msg)

        rate.sleep()

if __name__ == '__main__':   
     main()