#!/usr/bin/env python3
import rospy
from ai_controller.mosaic_controller import MosaicController
from cv_bridge import CvBridge
import cv2
from camera_controller.srv import *
import os
from ur5e_2f_85_controller.robotiq2f_85 import Robotiq2f85
import tf2_ros
import numpy as np
import math


def _quat2axisangle(self, quat):
    """
    Converts quaternion to axis-angle format.
    Returns a unit vector direction scaled by its angle in radians.
    Args:
        quat (np.array): (x,y,z,w) vec4 float angles
    Returns:
        np.array: (ax,ay,az) axis-angle exponential coordinates
    """
    # clip quaternion
    if quat[3] > 1.0:
        quat[3] = 1.0
    elif quat[3] < -1.0:
        quat[3] = -1.0

    den = np.sqrt(1.0 - quat[3] * quat[3])
    if math.isclose(den, 0.0):
        # This is (close to) a zero degree rotation, immediately return
        return np.zeros(3)

    return (quat[:3] * 2.0 * math.acos(quat[3])) / den


if __name__ == '__main__':

    rospy.init_node("ai_controller_node", log_level=rospy.INFO)

    # 1. Load Model
    import debugpy
    debugpy.listen(('0.0.0.0', 5678))
    print("Waiting for debugger attach")
    debugpy.wait_for_client()

    current_file_path = os.path.abspath(os.getcwd())
    model_folder = "checkpoint/mosaic"
    model_name = "model_save-29348.pt"

    conf_file_path = os.path.join(
        current_file_path, model_folder, "config.yaml")
    model_file_path = os.path.join(
        current_file_path, model_folder, model_name)

    # context path
    context_path = "/media/ciccio/Sandisk/multitask_dataset_baseline"
    context_robot_name = "sawyer"
    task_name = "pick_place"
    variation_number = 0
    trj_number = 0

    mosaic_controller = MosaicController(
        conf_file_path=conf_file_path,
        model_file_path=model_file_path,
        context_path=context_path,
        context_robot_name=context_robot_name,
        task_name=task_name,
        variation_number=variation_number,
        trj_number=trj_number)

    # 2. Camera srv proxy
    # camera client service
    rospy.loginfo("---- Waiting for env camera service ----")
    env_camera_service_client = rospy.ServiceProxy(
        "/get_frames", GetFrames, True)
    bridge = CvBridge()

    # 3. Initialise robot-gripper
    gripper = Robotiq2f85()

    # 4. Init tf listener for TCP Pose
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    exception = True
    while exception:
        try:
            # Get TCP Pose
            tcp_pose = tfBuffer.lookup_transform(
                'base_link', 'tcp_link', rospy.Time())
            rospy.logdebug(f"TCP Pose {tcp_pose}")
            exception = False
        except ((tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)) as e:
            exception = True
            rospy.logerr(e)

    while True:
        # 1. Get current observation
        env_frames = env_camera_service_client()
        color_cv_image = bridge.imgmsg_to_cv2(
            env_frames[0], desired_encoding='rgba8')
        color_cv_image = cv2.cvtColor(color_cv_image, cv2.COLOR_RGBA2RGB)
        # 2. Get current robot state
        # 2.1 Get ee_pose with respect to /base_link
        tcp_pose = tfBuffer.lookup_transform(
            'base_link', 'tcp_link', rospy.Time())
        pos = np.array(
            [tcp_pose.transform.translation.x,
             tcp_pose.transform.translation.y,
             tcp_pose.transform.translation.z])
        quat = np.array([tcp_pose.transform.rotation.x,
                         tcp_pose.transform.rotation.y,
                         tcp_pose.transform.rotation.z,
                         tcp_pose.transform.rotation.w])
        aa = _quat2axisangle(quat)
        # 2.2 Get Gripper joints positions
        finger_position = gripper.get_state()['finger_position']
        scaled_finger_position = finger_position/255
        left_joint = np.array([scaled_finger_position])
        right_joint = np.array([-scaled_finger_position])
        state = np.concatenate(
            arrays=(pos, quat, left_joint, right_joint))
        rospy.Rate.sleep(10)
