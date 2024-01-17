#!/usr/bin/env python3
import rospy
from ai_controller.mosaic_controller import MosaicController
from ai_controller.ctod_controller import CTODController
from cv_bridge import CvBridge
import cv2
from zed_camera_controller.srv import *
import os
from controller.robotiq2f_85 import Robotiq2f85
import tf2_ros
import numpy as np
import math
from move_group_python_interface import MoveGroupPythonInterface
from ur5e_2f_85_controller.srv import GoToJoint, GoToJointRequest

EPS = np.finfo(float).eps * 4.0
T_REAL_TO_SIM = np.array([[0, -1, 0, 0.40],
                          [1, 0, 0, -0.47],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1],])
T_G_SIM_TO_G_REAL_SIM = np.array([[0, 1, 0, 0.02],
                                  [-1, 0, 0, 0.0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1],])
max_T = 100


def _axisangle2quat(vec):
    """
    Converts scaled axis-angle to quat.
    Args:
        vec (np.array): (ax,ay,az) axis-angle exponential coordinates
    Returns:
        np.array: (x,y,z,w) vec4 float angles
    """
    # Grab angle
    angle = np.linalg.norm(vec)

    # handle zero-rotation case
    if math.isclose(angle, 0.0):
        return np.array([0.0, 0.0, 0.0, 1.0])

    # make sure that axis is a unit vector
    axis = vec / angle

    q = np.zeros(4)
    q[3] = np.cos(angle / 2.0)
    q[:3] = axis * np.sin(angle / 2.0)
    return q


def _quat2axisangle(quat):
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


def _mat2quat(rmat):
    """
    Converts given rotation matrix to quaternion.

    Args:
        rmat (np.array): 3x3 rotation matrix

    Returns:
        np.array: (x,y,z,w) float quaternion angles
    """
    M = np.asarray(rmat).astype(np.float32)[:3, :3]

    m00 = M[0, 0]
    m01 = M[0, 1]
    m02 = M[0, 2]
    m10 = M[1, 0]
    m11 = M[1, 1]
    m12 = M[1, 2]
    m20 = M[2, 0]
    m21 = M[2, 1]
    m22 = M[2, 2]
    # symmetric matrix K
    K = np.array(
        [
            [m00 - m11 - m22, np.float32(0.0),
             np.float32(0.0), np.float32(0.0)],
            [m01 + m10, m11 - m00 - m22, np.float32(0.0), np.float32(0.0)],
            [m02 + m20, m12 + m21, m22 - m00 - m11, np.float32(0.0)],
            [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
        ]
    )
    K /= 3.0
    # quaternion is Eigen vector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    inds = np.array([3, 0, 1, 2])
    q1 = V[inds, np.argmax(w)]
    if q1[0] < 0.0:
        np.negative(q1, q1)
    inds = np.array([1, 2, 3, 0])
    return q1[inds]


def _quat2mat(quaternion):
    """
    Converts given quaternion to matrix.

    Args:
        quaternion (np.array): (x,y,z,w) vec4 float angles

    Returns:
        np.array: 3x3 rotation matrix
    """
    # awkward semantics for use with numba
    inds = np.array([3, 0, 1, 2])
    q = np.asarray(quaternion).copy().astype(np.float32)[inds]

    n = np.dot(q, q)
    if n < EPS:
        return np.identity(3)
    q *= math.sqrt(2.0 / n)
    q2 = np.outer(q, q)
    return np.array(
        [
            [1.0 - q2[2, 2] - q2[3, 3], q2[1, 2] - q2[3, 0], q2[1, 3] + q2[2, 0]],
            [q2[1, 2] + q2[3, 0], 1.0 - q2[1, 1] - q2[3, 3], q2[2, 3] - q2[1, 0]],
            [q2[1, 3] - q2[2, 0], q2[2, 3] + q2[1, 0], 1.0 - q2[1, 1] - q2[2, 2]],
        ]
    )


def _convert_from_sim_space_to_real_sapce(sim_pos, sim_orientation):
    """Perform the conversion from pose obtained in simulated space into real-world space

    Args:
        sim_pos (_type_): _description_
        sim_orientation (_type_): _description_
    """
    R_sim_to_desired = _quat2mat(sim_orientation)
    sim_pos = np.reshape(sim_pos, (3, 1))
    T_sim_to_desired_sim = np.vstack(
        (np.hstack((R_sim_to_desired, sim_pos)), np.array([[0, 0, 0, 1]])))
    T_world_to_desired = (
        T_REAL_TO_SIM @ T_sim_to_desired_sim)  @ T_G_SIM_TO_G_REAL_SIM
    desired_pos = T_world_to_desired[:-1, 3]
    desired_orientation = _mat2quat(T_world_to_desired[:-1, :-1])
    return desired_pos, desired_orientation


if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_folder', type=str, default=None)
    parser.add_argument("--model_name", type=str, default=None)
    parser.add_argument("--context_path", type=str, default=None)
    parser.add_argument("--task_name", type=str, default=None)
    parser.add_argument("--context_robot_name", type=str, default=None)
    parser.add_argument("--variation_number", type=int, default=None)
    parser.add_argument("--trj_number", type=int, default=None)
    parser.add_argument("--debug", action="store_true")

    args, unknown = parser.parse_known_args()

    rospy.init_node("ai_controller_node", log_level=rospy.INFO)

    # 1. Load Model
    if args.debug:
        import debugpy
        debugpy.listen(('0.0.0.0', 5678))
        print("Waiting for debugger attach")
        debugpy.wait_for_client()

    current_file_path = os.path.dirname(os.path.abspath(__file__))
    model_folder = args.model_folder
    model_name = args.model_name
    conf_file_path = os.path.join(
        current_file_path, model_folder, "config.yaml")
    model_file_path = os.path.join(
        current_file_path, model_folder, model_name)
    # context path
    context_path = args.context_path
    context_robot_name = args.context_robot_name
    task_name = args.task_name
    variation_number = args.variation_number
    trj_number = args.trj_number

    rospy.loginfo(
        f"Loading the following AI-controller {model_folder.split('/')[-1]} - Step {model_file_path.split('/')[-1]}")
    if "mosaic" in model_folder and "ctod" not in model_folder:
        ai_controller = MosaicController(
            conf_file_path=conf_file_path,
            model_file_path=model_file_path,
            context_path=context_path,
            context_robot_name=context_robot_name,
            task_name=task_name,
            variation_number=variation_number,
            trj_number=trj_number,
            camera_name='camera_front')
    else:
        ai_controller = CTODController(
            conf_file_path=conf_file_path,
            model_file_path=model_file_path,
            context_path=context_path,
            context_robot_name=context_robot_name,
            task_name=task_name,
            variation_number=variation_number,
            trj_number=trj_number,
            camera_name='camera_front')

    # 2. Camera srv proxy
    # camera client service
    rospy.loginfo("---- Waiting for env camera service ----")
    env_camera_service_client = rospy.ServiceProxy(
        "/get_frames", GetFrames, False)
    bridge = CvBridge()

    cv2.namedWindow("Predicted bb", cv2.WINDOW_NORMAL)

    # 3. Initialise robot-gripper
    gripper = Robotiq2f85()
    move_group = MoveGroupPythonInterface(gripper_ref=gripper)

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

    # moveit service
    home_pos = [1.636146903038025, -1.7759367428221644, 2.2431824843036097, -
                2.017546316186422, 4.7087507247924805, 0.002162999240681529]
    # wait for going in home position
    enter = None
    while enter != "":
        rospy.loginfo("Press enter to go to gome: ")
        enter = input()
    # ask for going to home position
    home_pos = np.array([-0.156, 0.349, 0.153])
    home_quat = np.array([0.999, 0.032, 0.002, 0.010])
    home_gripper_pos = 0
    rospy.loginfo(f"Position {home_pos} - Gripper pos {home_gripper_pos}")
    result = move_group.go_to_pose_goal(
        position=home_pos, orientation=home_quat, gripper_pos=home_gripper_pos)
    if result:
        rospy.loginfo_once(
            "Robot in home position, ready to get a new trajectory")

    while True:
        task_number = int(input("Insert the task number: "))
        trj_number = int(input("Insert the trj number: "))
        rospy.loginfo(
            f"Test task: {task_number} - Variation {variation_number}")
        ai_controller.modify_context(
            variation_number=task_number,
            trj_number=trj_number)
        t = 0
        while t < max_T:
            # 1. Get current observation
            env_frames = env_camera_service_client()
            color_cv_image = bridge.imgmsg_to_cv2(
                env_frames.color_frames[0],
                desired_encoding='rgba8')
            color_cv_image = cv2.cvtColor(
                np.array(color_cv_image), cv2.COLOR_RGBA2RGB)
            # resize image
            cv2.imwrite(os.path.join(os.path.dirname(
                os.path.abspath(__file__)), "original.png"), color_cv_image)
            # dim = (180, 100)
            # color_cv_image = cv2.resize(
            #     color_cv_image, dim, interpolation=cv2.INTER_AREA)
            # cv2.imwrite("/catkin_ws/src/Ur5e-2f-85f/resized.png", color_cv_image)
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
            left_joint = np.array(scaled_finger_position)
            right_joint = np.array(-scaled_finger_position)
            state = np.concatenate(
                (pos, aa, [finger_position]))

            # 3. Run action inference
            action, predicted_bb = ai_controller.get_action(obs=color_cv_image,
                                                            robot_state=state)

            cv2.imshow("Predicted bb", predicted_bb)
            cv2.waitKey(500)
            # 4. Perform action
            # 4.1 Decopose action
            desired_position = action[:3]
            desired_orientation = _axisangle2quat(vec=action[3:6])
            predicted_gripper = action[-1]
            if predicted_gripper > 0.75:
                gripper_finger_pos = 255
            else:
                gripper_finger_pos = 0
            # 4.2 Call controller
            rospy.loginfo(
                f"Predicted action: {desired_position}, {desired_orientation}, {gripper_finger_pos}")
            # desired_position, desired_orientation = _convert_from_sim_space_to_real_sapce(
            #     sim_pos=desired_position, sim_orientation=desired_orientation)
            rospy.loginfo(
                f"Real-world desired pose: {desired_position}, {desired_orientation}, {gripper_finger_pos}")
            move_group.go_to_pose_goal(position=desired_position,
                                       orientation=desired_orientation,
                                       gripper_pos=gripper_finger_pos)

            rospy.Rate(1).sleep()
            t += 1

        rospy.loginfo(
            f"Summary task: {task_number} - Variation {variation_number}")
