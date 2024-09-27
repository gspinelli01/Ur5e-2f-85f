#!/usr/bin/env python3
import rospy
import pickle as pkl

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from controller.robotiq2f_85 import Robotiq2f85
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

PKL_FILE_PATH = "/media/ciccio/Sandisk/real-world-dataset/pick_place/task_00/traj000.pkl"


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self, gripper_ref=None):
        super(MoveGroupPythonInterface, self).__init__()

        # BEGIN_SUB_TUTORIAL setup
        ##
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "tcp_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        self._controller_manager_state_srv = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers)

        self._switch_controller_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController)


        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        move_group.set_pose_reference_frame('base_link')
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        # END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        if gripper_ref is not None:
            self._gripper = gripper_ref
        else:
            self._gripper = Robotiq2f85()

    def go_to_joint_state(self, joint_goal_pos=[]):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joint_goal_pos[0]
        joint_goal[1] = joint_goal_pos[1]
        joint_goal[2] = joint_goal_pos[2]
        joint_goal[3] = joint_goal_pos[3]
        joint_goal[4] = joint_goal_pos[4]
        joint_goal[5] = joint_goal_pos[5]

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def _start_stop_controllers(self, start=[], stop=[]):
        # check running controller
        controllers_state = self._controller_manager_state_srv.call()
        switch_controller_req = SwitchControllerRequest()
        controller_to_start = []
        controller_to_stop = []
        for controller in controllers_state.controller:
            # if pos_joint_traj_controller is stopped then start it
            if (controller.name in start) and controller.state == 'stopped':
                controller_to_start.append(controller.name)
            elif (controller.name in stop) and controller.state == 'running':
                controller_to_stop.append(controller.name)

        switch_controller_req.strictness = switch_controller_req.BEST_EFFORT
        switch_controller_req.start_controllers = controller_to_start
        switch_controller_req.stop_controllers = controller_to_stop
        switch_res = self._switch_controller_srv.call(switch_controller_req)
        if switch_res.ok:
            # rospy.loginfo("scaled_pos_joint_traj_controller started correctly")
            return switch_res.ok
        else:
            # rospy.logerr("scaled_pos_joint_traj_controller not started")
            return switch_res.ok

    def go_to_pose_goal(self, position=[], orientation=[], gripper_pos=-1):
        if self._start_stop_controllers(start=["scaled_pos_joint_traj_controller"], stop=["twist_controller"]):
            move_group = self.move_group
            waypoints = []

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = float(orientation[0])
            pose_goal.orientation.y = float(orientation[1])
            pose_goal.orientation.z = float(orientation[2])
            pose_goal.orientation.w = float(orientation[3])
            pose_goal.position.x = float(position[0])
            pose_goal.position.y = float(position[1])
            pose_goal.position.z = float(position[2])

            waypoints.append(copy.deepcopy(pose_goal))
            (plan, fraction) = move_group.compute_cartesian_path(waypoints,
                                                                0.01,
                                                                0.0)
            # Display compute path
            display_trj = moveit_msgs.msg.DisplayTrajectory()
            display_trj.trajectory_start = self.robot.get_current_state()
            display_trj.trajectory.append(plan)
            self.display_trajectory_publisher.publish(display_trj)

            if fraction == 1:
                # press = input("Y to perform planned trajectory:")
                enter = None
                # while enter != "":
                #     rospy.loginfo("Press enter to go to next wp: ")
                #     enter = input()
                if True:  # press == 'Y' or press == 'y':
                    success = move_group.execute(plan, wait=True)

                    move_group.set_pose_target(pose_goal)

                    self._gripper.send_command(
                        command='s', position=gripper_pos, force=100, speed=255)
                    rospy.logdebug(f"Move group result {success}")
                    if success:
                        # Calling `stop()` ensures that there is no residual movement
                        move_group.stop()
                        # It is always good to clear your targets after planning with poses.
                        # Note: there is no equivalent function for clear_joint_value_targets().
                        move_group.clear_pose_targets()

                        # END_SUB_TUTORIAL

                        # For testing:
                        # Note that since this section of code will not be included in the tutorials
                        # we use the class variable rather than the copied state variable
                        # current_pose = self.move_group.get_current_pose().pose
                        # rospy.loginfo(f"{self.move_group.get_current_pose()}")
                        # return all_close(pose_goal, current_pose, 0.1)
                    return success
                else:
                    rospy.loginfo("Execution blocked")
                    return False
            else:
                rospy.logerr(f"Planning fraction not 1: {fraction}")
                return False
