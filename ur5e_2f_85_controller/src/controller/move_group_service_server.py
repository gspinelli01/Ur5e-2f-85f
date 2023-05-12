#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs
from ur5e_2f_85_controller.srv import GoToJoint, GoToJointResponse
from math import pi, tau, dist, fabs, cos
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest

class MoveGroupServiceServer():

    def __init__(self, group_name):
        super(MoveGroupServiceServer, self).__init__()

        # First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)
        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()
        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()
        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # MoveGroupCommander
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.go_to_joint_srv_server = rospy.Service(
            "/go_to_joint", GoToJoint, self.go_to_joint)

        self._controller_manager_state_srv = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers)

        self._switch_controller_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController)

    def all_close(self, goal, actual, tolerance):
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
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
            # Euclidean distance
            d = dist((x1, y1, z1), (x0, y0, z0))
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

        return True

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
            rospy.loginfo("scaled_pos_joint_traj_controller started correctly")
            return switch_res.ok
        else:
            rospy.logerr("scaled_pos_joint_traj_controller not started")
            return switch_res.ok

    def go_to_joint(self, request):
        stop_controller = False
        if self._start_stop_controllers(start=["scaled_pos_joint_traj_controller"], stop=["twist_controller"]):
            # Get the joint values from the group and change some of the values:
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = request.joint_goal_pos[0]
            joint_goal[1] = request.joint_goal_pos[1]
            joint_goal[2] = request.joint_goal_pos[2]
            joint_goal[3] = request.joint_goal_pos[3]
            joint_goal[4] = request.joint_goal_pos[4]
            joint_goal[5] = request.joint_goal_pos[5]
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()

            # check the execution results
            current_joints = self.move_group.get_current_joint_values()
            success = self.all_close(joint_goal, current_joints, 0.01)
            if success:
                response = GoToJointResponse(
                    success=success, msg="Robot in desired position")
            else:
                response = GoToJointResponse(
                    success=success, msg="Robot not in desired position")

            if stop_controller:
                self._start_stop_controllers(start=["twist_controller"], stop=[
                    "scaled_pos_joint_traj_controller"])
            return response
        else:
            rospy.logerr("scaled_pos_joint_traj_controller not started")
            return GoToJointResponse(success=False, msg="scaled_pos_joint_traj_controller not started")


if __name__ == '__main__':

    rospy.init_node("move_group_service_server", anonymous=True)
    move_group_service_server = MoveGroupServiceServer(group_name="tcp_group")
    rospy.spin()
