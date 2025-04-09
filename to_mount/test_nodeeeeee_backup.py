#!/usr/bin/env python3
import cv2
cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
from pynput import keyboard
import json
from multi_task_il.datasets.savers import Trajectory
import torch
from sklearn.metrics import mean_squared_error
import random
import pickle as pkl
from sensor_msgs.msg import JointState
from move_group_python_interface import MoveGroupPythonInterface
import math
import numpy as np
import tf2_ros
import os
from zed_camera_controller.srv import *
from cv_bridge import CvBridge
from ai_controller.ctod_controller import CTODController
from ai_controller.mosaic_controller import MosaicController
from ai_controller.kp_controller import KPController
import rospy
from pynput import keyboard
from copy import deepcopy
print(f"cv2 file {cv2.__file__}")


STOP = False
EPS = np.finfo(float).eps * 4.0
T_REAL_TO_SIM = np.array([[0, -1, 0, 0.40],
                          [1, 0, 0, -0.47],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1],])
T_G_SIM_TO_G_REAL_SIM = np.array([[0, 1, 0, 0.02],
                                  [-1, 0, 0, 0.0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1],])
max_T = 200


def seed_everything(seed=42):
    print(f"Cuda available {torch.cuda.is_available()}")
    random.seed(seed)
    np.random.seed(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    "sxyz": (0, 0, 0, 0),
    "sxyx": (0, 0, 1, 0),
    "sxzy": (0, 1, 0, 0),
    "sxzx": (0, 1, 1, 0),
    "syzx": (1, 0, 0, 0),
    "syzy": (1, 0, 1, 0),
    "syxz": (1, 1, 0, 0),
    "syxy": (1, 1, 1, 0),
    "szxy": (2, 0, 0, 0),
    "szxz": (2, 0, 1, 0),
    "szyx": (2, 1, 0, 0),
    "szyz": (2, 1, 1, 0),
    "rzyx": (0, 0, 0, 1),
    "rxyx": (0, 0, 1, 1),
    "ryzx": (0, 1, 0, 1),
    "rxzx": (0, 1, 1, 1),
    "rxzy": (1, 0, 0, 1),
    "ryzy": (1, 0, 1, 1),
    "rzxy": (1, 1, 0, 1),
    "ryxy": (1, 1, 1, 1),
    "ryxz": (2, 0, 0, 1),
    "rzxz": (2, 0, 1, 1),
    "rxyz": (2, 1, 0, 1),
    "rzyz": (2, 1, 1, 1),
}

def vec(values):
    """
    Converts value tuple into a numpy vector.

    Args:
        values (n-array): a tuple of numbers

    Returns:
        np.array: vector of given values
    """
    return np.array(values, dtype=np.float32)

def _mat2euler(rmat, axes="sxyz"):
    """
    Converts given rotation matrix to euler angles in radian.

    Args:
        rmat (np.array): 3x3 rotation matrix
        axes (str): One of 24 axis sequences as string or encoded tuple (see top of this module)

    Returns:
        np.array: (r,p,y) converted euler angles in radian vec3 float
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    M = np.asarray(rmat, dtype=np.float32)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j] * M[i, j] + M[i, k] * M[i, k])
        if sy > EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy, M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i] * M[i, i] + M[j, i] * M[j, i])
        if cy > EPS:
            ax = math.atan2(M[k, j], M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2(M[j, i], M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return vec((ax, ay, az))


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


def go_home(move_group):
    enter = None
    while enter != "":
        rospy.loginfo("Press enter to go to gome: ")
        enter = input()
    # ask for going to home position
    home_pos = np.array(
        [-0.15553531642007706, 0.34870957566273, 0.15331161752798533])
        # [-0.03553531642007706, 0.34870957566273, 0.15331161752798533])
    home_quat = np.array(
        [0.9994464628837594, 0.0315916758512133, 0.0021489054033469, 0.010203727339619493])
    home_gripper_pos = 0
    rospy.loginfo(f"Position {home_pos} - Gripper pos {home_gripper_pos}")
    result = move_group.go_to_pose_goal(
        position=home_pos, orientation=home_quat, gripper_pos=home_gripper_pos)
    if result:
        rospy.loginfo_once(
            "Robot in home position, ready to get a new trajectory")


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


def check_pick(predicited_gripper_pos, current_gripper_pos):
    if predicited_gripper_pos == 255 and current_gripper_pos < 230:
        return True
    return False


def save_record(save_path, model_name, task_name, task_number, variation, trj, context, cnt, res_dict):
    res_path = os.path.join(save_path, "test_res", model_name, task_name, 'task_{:02d}'.format(task_number))
    os.makedirs(res_path, exist_ok=True)

    rospy.loginfo(f"Saving res dict")
    json_file_path = os.path.join(res_path, f"traj{cnt}.json")
    json.dump(res_dict, open(json_file_path, 'w'))

    rospy.loginfo(f"Saving trajectory and context")
    pkl.dump(trj, open(
        res_path+f'/traj{cnt}.pkl', 'wb'))
    pkl.dump(context, open(
        res_path+f'/context{cnt}.pkl', 'wb'))


def on_press(key):
    rospy.loginfo(f"Pressing key {key.char}")
    if key.char == 'r':
        rospy.loginfo(f"Reset requested")
        global STOP
        STOP = True
        
        go_home()
        

def write_summary(model_name, task_number, variation_number):
    rospy.loginfo(
    f"Summary task: {task_number} - Variation {variation_number}")
    key = ''   
    valid = False
    while not valid:
        try:
            rospy.loginfo("Object reached? [1: True, 0: False]: ")
            key = input()
            reached = int(key)
            valid = True
        except:
            pass
    key = ''   
    valid = False
    while not valid:
        try:
            rospy.loginfo("Object picked? [1: True, 0: False]: ")
            key = input()
            picked = int(key)
            valid = True
        except:
            pass
    key = ''   
    valid = False
    while not valid:
        try:
            rospy.loginfo("Object placed? [1: True, 0: False]: ")
            key = input()
            placed = int(key)
            valid = True
        except:
            pass
    key = ''   
    valid = False
    while not valid:
        try:
            rospy.loginfo("Reached wrong? [1: True, 0: False]: ")
            key = input()
            reached_wrong = int(key)
            valid = True
        except:
            pass
    key = ''   
    valid = False
    while not valid:
        try:
            rospy.loginfo("Wrong object correct bin? [1: True, 0: False]: ")
            key = input()
            wrong_obj_corr_bin = int(key)
            valid = True
        except:
            pass
    key = ''   
    valid = False
    while not valid:
        try:
            rospy.loginfo("Correct object wrong bin? [1: True, 0: False]: ")
            key = input()
            corr_obj_wrong_bin = int(key)
            valid = True
        except:
            pass
    key = ''   
    valid = False
    while not valid:
        try:
            rospy.loginfo("Wrong both? [1: True, 0: False]: ")
            key = input()
            wrong_obj_wrong_bin = int(key)
            valid = True
        except:
            pass


    res_dict = dict()
    res_dict['success'] = placed
    res_dict['reached'] = reached
    res_dict['picked'] = picked
    res_dict['reached_wrong'] = reached_wrong
    res_dict['variation_id'] = variation_number
    res_dict['wrong_obj_corr_bin'] = wrong_obj_corr_bin
    res_dict['corr_obj_wrong_bin'] = corr_obj_wrong_bin
    res_dict['wrong_obj_wrong_bin'] = wrong_obj_wrong_bin


    save_record(save_path=args.save_path,
                model_name=model_name,
                task_name=task_name,
                task_number=task_number,
                variation=variation_number,
                trj=traj,
                context=ai_controller._context,
                res_dict=res_dict,
                cnt=cnt)

    

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
    parser.add_argument("--save_path", type=str,
                        default="/media/ciccio/Sandisk/")
    parser.add_argument("--correct_sample", action='store_true')
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--write_summary", action="store_true")
    
    args, unknown = parser.parse_known_args()

    rospy.init_node("ai_controller_node", log_level=rospy.INFO)

    # 1. Load Model
    if args.debug:
        import debugpy
        debugpy.listen(('0.0.0.0', 5678))
        print("Waiting for debugger attach")
        debugpy.wait_for_client()

    seed_everything()
    cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
    cv2.moveWindow('Image', 25, 350)

    current_file_path = os.path.dirname(os.path.abspath(__file__))
    model_folder = args.model_folder
    model_name = args.model_name
    # model_char = model_name.split("model_save")[-1].split("-")[-1]
    model_char = 'model_save'
    rospy.loginfo(f"Testing model with char {model_char}")
    conf_file_path = os.path.join(
        current_file_path, model_folder, f"config_{model_char}.yaml")
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
    if "mosaic" in model_folder and "_ctod" not in model_folder and '_kp' not in model_folder:
        ai_controller = MosaicController(
            conf_file_path=conf_file_path,
            model_file_path=model_file_path,
            context_path=context_path,
            context_robot_name=context_robot_name,
            task_name=task_name,
            variation_number=variation_number,
            trj_number=trj_number,
            camera_name='camera_front')
    elif "mosaic_ctod" in model_folder:
        ai_controller = CTODController(
            conf_file_path=conf_file_path,
            model_file_path=model_file_path,
            context_path=context_path,
            context_robot_name=context_robot_name,
            task_name=task_name,
            variation_number=variation_number,
            trj_number=trj_number,
            camera_name='camera_front')
    elif "mosaic_kp" in model_folder or "MOSAIC-KP" in model_folder:
        ai_controller = KPController(
            conf_file_path=conf_file_path,
            model_file_path=model_file_path,
            context_path=context_path,
            context_robot_name=context_robot_name,
            task_name=task_name,
            variation_number=variation_number,
            trj_number=trj_number,
            camera_name='camera_front')
    elif "rt1_real" in model_folder:

        from ai_controller.RT1_controller import RT1Controller
        ai_controller = RT1Controller(
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

    # cv2.namedWindow("Predicted bb", cv2.WINDOW_NORMAL)
    move_group = MoveGroupPythonInterface()

    if "rt1_real" in model_folder:
        # 4. Init tf listener for TCP Pose
        tfBuffer = tf2_ros.Buffer()
        listener_rt1 = tf2_ros.TransformListener(tfBuffer)
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
    else:
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

    enter = None
    cnt = 1000
        
    while True:
        rospy.loginfo(f"Starting rollout number {cnt}")
        if isinstance(ai_controller, KPController):
            ai_controller.change_phase(first_phase=True)
        
        go_home(move_group=move_group)
        valid = False
        while not valid:
            try:
                task_number = int(input("Insert the task number: "))
                valid = True
            except:
                pass
        
        if task_number == -1:
            rospy.loginfo("Exit demo mode")
            rospy.signal_shutdown("User requested signal_shutdown")

        valid = False
        while not valid:
            try:
                trj_number = int(input("Insert the trj number: "))
                valid = True
            except:
                pass
        rospy.loginfo(
            f"Test task: {task_number} - Variation {variation_number}")
        ai_controller.modify_context(
            variation_number=task_number,
            trj_number=trj_number)

        t = 0
        ai_controller.reset_timer() ##########################
        rospy.loginfo(f"Controller timer: {ai_controller._t}")
        picked_old = False
        picked = False
        trj = None
        done = False
        traj = Trajectory()
        STOP = False
        ################################
        TRAINING_DOMAIN = False
        DELTA_ACTION = False
        MOSAIC_CTOD = True
        MOSAIC_WITH_STATE = False
        STUCK_TROUBLESHOOT = False
        STUCK_PATIENCE = 5
        stuck_counter = 0
        old_norm = None
        if TRAINING_DOMAIN:
            with open("/home/gianl/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/checkpoint/real_new_ur5e_pick_place_converted_absolute/task_14/traj000.pkl", "rb") as f:
                sample = pkl.load(f)
            trj = sample['traj'] 
        ################################
            # Initialize the keyboard listener
        rospy.loginfo("Starting keyboad listener")
        listener = None
        while t < max_T and not done and not STOP:
            if listener is None:
                listener = keyboard.Listener(
                on_press=on_press)
            else:
                listener.stop()            
            listener = keyboard.Listener(
            on_press=on_press)
            listener.start()
        
            obs = dict()
            # 1. Get current observation
            if trj is not None:
                color_cv_image = np.array(trj.get(
                    t)['obs'][f'camera_front_image'])
                gt_action = trj.get(
                    t)['action']
                rospy.loginfo(f"GT_ACTION {gt_action[:3]}")
                # next_action = trj.get(
                #     t+10)['action']
                # rospy.loginfo(f"{gt_action[:3]-next_action[:3]}")
                state = np.concatenate((trj.get(
                    t)['obs']['joint_pos'], trj.get(
                    t)['obs']['joint_vel']))

                data = rospy.wait_for_message(
                    '/joint_states',
                    JointState)
                joint_pos = np.array(data.position)
                joint_vel = np.array(data.velocity)
                state_robot = np.concatenate((joint_pos, joint_vel))
                rospy.logerr(f"\nState obs {state}\nState robot {state_robot}")

            else:
                env_frames = env_camera_service_client()
                color_cv_image = bridge.imgmsg_to_cv2(
                    env_frames.color_frames[0],
                    desired_encoding='rgba8')
                color_cv_image = cv2.cvtColor(
                    np.array(color_cv_image), cv2.COLOR_RGBA2RGB)
                
                # depth_cv_image = bridge.imgmsg_to_cv2(
                #                         env_frames.depth_frames[0], 
                #                         desired_encoding='passthrough')

                # state = np.concatenate((trj.get(
                #     t)['obs']['joint_pos'], trj.get(
                #     t)['obs']['joint_vel']))
                # rospy.loginfo(f"Trj state {trj.get(t)['obs']['joint_pos']}")


                if True: #"rt1_real" in model_folder:
                    # here the state is made up of the pos and rot of tcp wrt bl    
                    try:
                        # Get TCP Pose
                        tcp_pose = tfBuffer.lookup_transform(
                            'base_link', 'tcp_link', rospy.Time())
                        rospy.logdebug(f"TCP Pose {tcp_pose}")
                    except ((tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)) as e:
                        rospy.logerr(e)
                            
                    if t == 0:
                        gripper_state = [0]
                    else:
                        gripper_state = [action[-1]]

                    tcp_pose_numpy = np.array([tcp_pose.transform.translation.x, tcp_pose.transform.translation.y, tcp_pose.transform.translation.z])
                    tcp_rot_quat_numpy = np.array([tcp_pose.transform.rotation.x, tcp_pose.transform.rotation.y, tcp_pose.transform.rotation.z, tcp_pose.transform.rotation.w])
                    # rospy.loginfo(f"Robot tcp pose wr to bl: {tcp_pose_numpy}")
                    # rospy.loginfo(f"Robot tcp rot wr to bl: {tcp_rot_quat_numpy}")

                    tcp_rot_euler_numpy = _mat2euler(_quat2mat(tcp_rot_quat_numpy))

                    if not MOSAIC_WITH_STATE:
                        state = np.concatenate((tcp_pose_numpy, tcp_rot_euler_numpy, gripper_state))


                if MOSAIC_WITH_STATE:
                    data = rospy.wait_for_message(
                        '/joint_states',
                        JointState)
                    joint_pos = np.array(data.position)
                    shoulder_pan_joint_pos = joint_pos[2]
                    joint_pos[2] = joint_pos[0]
                    joint_pos[0] = shoulder_pan_joint_pos
                    if t == 0:
                        gripper_state = [0]
                    else:
                        gripper_state = [action[-1]]
                    # rospy.loginfo(f"Robot state {joint_pos}")
                    # joint_vel = np.array(data.velocity)
                    # state = np.concatenate((joint_pos, joint_vel))
                    state = np.concatenate((joint_pos, gripper_state))

            # resize image
            cv2.imwrite(os.path.join(os.path.dirname(
                os.path.abspath(__file__)), "original_RGB.png"), color_cv_image[:,:,::-1])
            original_image = deepcopy(color_cv_image)
            # 3. Run action inference
            if "rt1_real" in model_folder:
                action, image = ai_controller.get_action(obs=color_cv_image,
                                                                    robot_state=state)
            else:
                action, predicted_bb, image = ai_controller.get_action(obs=color_cv_image,
                                                                    robot_state=state)
            

            obs['camera_front_image'] = image
            # obs['camera_front_image'] = image
            obs['camera_front_image_full_size'] = original_image
            # obs['camera_front_depth_full_size'] = deepcopy(depth_cv_image)
            if "rt1_real" not in model_folder:
                shoulder_pan_joint_pos = state[0]
                state[0] = state[2]
                state[2] = shoulder_pan_joint_pos
            obs['state'] = state
            obs['action'] = action
            if "rt1_real" not in model_folder and predicted_bb is not None:
                obs['predicted_bb'] = dict()
                obs['predicted_bb']['camera_front'] = predicted_bb
            traj.append(obs)

            # 4. Perform action
            # 4.1 Decopose action
            desired_position = action[:3]
            rospy.loginfo(f"DESIRED_POSITION: {desired_position}")
            if False and trj is not None:
                # error_t = mean_squared_error(y_true=np.array([gt_action[:3]]),
                #                              y_pred=np.array([action[:3]]))
                # error_t = np.linalg.norm(
                #     [gt_action[:3]] - np.array([action[:3]]), axis=1)
                error_t = gt_action[:3]-action[:3]
                rospy.logerr(f"Error {error_t}")
                if abs(error_t[2]) > 0.01:
                    print(f"Error z {error_t}")
            # _axisangle2quat(vec=action[3:6])
            # np.array([0.999, 0.032, 0.002, 0.010])

            #### ORIENTATION
            # desired_orientation = np.array([0.999, 0.032, 0.002, 0.010])
            desired_orientation = action[3:-1]
            #############

            #_axisangle2quat(vec=action[3:6])
            predicted_gripper = int(action[-1]*255)
            if predicted_gripper < 0:
                predicted_gripper = 0
            gripper_finger_pos = predicted_gripper
            # rospy.loginfo(
            #     f"Predicted action: {desired_position}, {desired_orientation}, {gripper_finger_pos}")

            rospy.loginfo("-----------------------------------------------------------------------------")
            # rospy.loginfo(f"Current pos: {tcp_pose_numpy.round(3)}")
            # rospy.loginfo(f"Desired pose: {desired_position.round(3)}")
            # rospy.loginfo(f"Current RPY: {tcp_rot_euler_numpy.round(3)}")
            # rospy.loginfo(f"Desired RPY: {_mat2euler(_quat2mat(desired_orientation)).round(3)}")
            # rospy.loginfo(f"Current gripper: {gripper_state}")
            # rospy.loginfo(f"Desired gripper: {gripper_finger_pos}")
            if not TRAINING_DOMAIN and not DELTA_ACTION and not MOSAIC_CTOD:
                rospy.loginfo(f"Diff pos: {(desired_position - tcp_pose_numpy).round(3)}")
                rospy.loginfo(f"Diff RPY: {(_mat2euler(_quat2mat(desired_orientation)) - tcp_rot_euler_numpy).round(3)}")
                rospy.loginfo(f"Diff gripper: {(gripper_finger_pos % 254) - gripper_state[0]}")
                rospy.loginfo(f"Stuck counter: {stuck_counter}")

            if MOSAIC_CTOD:
                if desired_position[2] < 0.10 and not picked:
                    rospy.loginfo("Adding offset during reaching")
                    desired_position[0] = desired_position[0] + 0.02
                # desired_position[2] -= 0.01
            if not TRAINING_DOMAIN and not DELTA_ACTION and not MOSAIC_CTOD:
                if t < -1 or ((desired_position - tcp_pose_numpy).round(3) > 0.45).any():
                    desired_position = tcp_pose_numpy
                    desired_orientation = tcp_rot_quat_numpy
                    gripper_finger_pos = int(gripper_state[0])
                # else:
                #     desired_orientation = tcp_rot_quat_numpy # fix rotation
                if desired_position[1] - tcp_pose_numpy[1] < -0.2:
                    desired_position = tcp_pose_numpy
                    desired_orientation = tcp_rot_quat_numpy
                    gripper_finger_pos = int(gripper_state[0])
                    rospy.loginfo(f"Ur5 wants to go back!!!")

            
            if STUCK_TROUBLESHOOT and not MOSAIC_CTOD:
                # if gripper is still stuck, increase counter
                if np.linalg.norm((desired_position - tcp_pose_numpy).round(3)) == 0.0:
                    stuck_counter+=1
                
                if stuck_counter >= STUCK_PATIENCE:
                    # move up or down
                    if desired_position[2] > 0.10:
                        desired_position[2] = desired_position[2] - 0.05 
                    elif desired_position[2] > -0.03 and desired_position[2] <= 0.10:
                        desired_position[2] = desired_position[2] - 0.03 
                    else:
                        desired_position[2] = desired_position[2] + 0.05 
                    # reset counter
                    stuck_counter = 0

                # reset counter if there's movement
                if np.linalg.norm((desired_position - tcp_pose_numpy).round(3)) != 0.0:
                    stuck_counter = 0

                # if "task_00" in 
            # if task_number == 0:
            #     desired_position[0] = desired_position[0] + 0.02

            # FOR MOSAIC CTOD
            # desired_orientation = _axisangle2quat(desired_orientation)


            # if (abs((desired_position - tcp_pose_numpy).round(3)) > 0.20).any():
            #     desired_position = tcp_pose_numpy
            #     desired_orientation = tcp_rot_quat_numpy
            #     gripper_finger_pos = int(gripper_state[0])
            #     res = move_group.go_to_pose_goal(position=desired_position,
            #                 orientation=desired_orientation,
            #                 gripper_pos=gripper_finger_pos)
            # else:

            # if t > -1:
            #     desired_orientation = tcp_rot_quat_numpy
            if DELTA_ACTION:
                desired_position = desired_position + tcp_pose_numpy
                # desired_orientation = desired_orientation + tcp_rot_quat_numpy
                desired_orientation = tcp_rot_quat_numpy

            if MOSAIC_CTOD:
                desired_orientation = tcp_rot_quat_numpy
                res = move_group.go_to_pose_goal(position=desired_position,
                                        orientation=desired_orientation,
                                        gripper_pos=gripper_finger_pos)
            else:
                res = move_group.go_to_pose_goal(position=desired_position,
                                        orientation=desired_orientation,
                                        gripper_pos=gripper_finger_pos)
            # trick
            if gripper_finger_pos == 255 and not picked:
                rospy.Rate(1).sleep()
                
            
            if not res:
                STOP = True
                
            rospy.Rate(10).sleep()

            # check if the object has been picked
            if gripper_finger_pos == 255 and not picked and not picked_old:
                picked = check_pick(predicited_gripper_pos=gripper_finger_pos,
                                    current_gripper_pos=move_group._gripper.get_state()['finger_position'])
                # transition between close and open
                if picked and not picked_old:
                    rospy.loginfo("Transition open->close: Object pickded")
                    picked_old = picked
                    if MOSAIC_CTOD:
                        desired_position[2] = desired_position[2] + 0.05 
                        move_group.go_to_pose_goal(position=desired_position,
                                        orientation=desired_orientation,
                                        gripper_pos=gripper_finger_pos)
                else:
                    rospy.loginfo("Transition open->close: Object not picked")

            # check for task completion
            if gripper_finger_pos < 20:
                if picked:
                    rospy.loginfo("Transition close->open")
                    key = ''
                    while key != "1" and key != "0":
                        rospy.loginfo("Done? [1: True, 0: False]: ")
                        key = input()
                        done = bool(int(key))

                        if done == False:
                            picked = False

            t += 1

        if not STOP:
            rospy.loginfo("Trajectory completed - Move robot in safe location")
            if done:
                desired_position[2] = desired_position[2] + 0.10
                gripper_finger_pos = 0
                move_group.go_to_pose_goal(position=desired_position,
                                        orientation=desired_orientation,
                                        gripper_pos=gripper_finger_pos)
            go_home(move_group=move_group)
        if STOP:
            rospy.loginfo("Stop requested")
        
        
        if args.write_summary:
            write_summary(
                model_name=model_name.split('.')[0],
                task_number=task_number,
                        variation_number=variation_number)
            cnt += 1
        
        if args.correct_sample:
            import dataset_collector 
            from incremental_procedure import collect_correct_sample


            enter=""
            while enter != 'Y' and enter != 'N':
                rospy.loginfo("Do you want to collect correct sample [Y/N]: ")
                enter = input()
            
            if enter == 'Y':
                rospy.loginfo(f"---- Collecting correct sample for task-id {task_number} Numeber {cnt}----")
                collect_correct_sample(
                                model_name=model_name.split('.')[0],
                                task_name="pick_place", 
                                task_id=task_number, 
                                start_trj_cnt= cnt-1,
                                traj=traj)
            
            else:
                rospy.loginfo("---- Continue with test ----")


                