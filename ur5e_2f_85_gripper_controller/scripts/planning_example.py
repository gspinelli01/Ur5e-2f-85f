# This code is based on the MoveIt tutorial (https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

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

def generate_command_msg(action):
    
    def genCommand(char, command):
        """Update the command according to the character entered by the user."""    
            
        if char == 'a':
            command = outputMsg.Robotiq2FGripper_robot_output();
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255
            command.rFR  = 150

        if char == 'r':
            command = outputMsg.Robotiq2FGripper_robot_output();
            command.rACT = 0

        if char == 'c':
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255
            command.rFR  = 150
            command.rPR = 255

        if char == 'o':
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255
            command.rFR  = 150
            command.rPR = 0   

        #If the command entered is a int, assign this value to rPRA
        try: 
            command.rPR = int(char)
            if command.rPR > 255:
                command.rPR = 255
            if command.rPR < 0:
                command.rPR = 0
        except ValueError:
            pass                    
            
        if char == 'f':
            command.rSP += 25
            if command.rSP > 255:
                command.rSP = 255
                
        if char == 'l':
            command.rSP -= 25
            if command.rSP < 0:
                command.rSP = 0

                
        if char == 'i':
            command.rFR += 25
            if command.rFR > 255:
                command.rFR = 255
                
        if char == 'd':
            command.rFR -= 25
            if command.rFR < 0:
                command.rFR = 0

        return command

    command = outputMsg.Robotiq2FGripper_robot_output()
    # generate command based on the required action
    command = genCommand(action, command)     
    rospy.loginfo(f"\n{command}")
    return command

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("planning_example_node", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "tcp_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
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
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, joint_goal_pos=[]):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joint_goal_pos[0]
        joint_goal[1] = joint_goal_pos[1]
        joint_goal[2] = joint_goal_pos[2]
        joint_goal[3] = joint_goal_pos[3]
        joint_goal[4] = joint_goal_pos[4]
        joint_goal[5] = joint_goal_pos[5]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position=[], orientation=[]):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


def main():

    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface  for UR5e")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    input(
        "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    )
    movegroup_interface = MoveGroupPythonInterfaceTutorial()
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    goal_position = []
    goal_orientation = []

    input(
        "============ Press `Enter` to go the the home position ..."
    )
    movegroup_interface.go_to_joint_state([1.59, -3.08, 2.68, -2.69, 4.63, 0.0435])
    input(
        "============ Press `Enter` to go the the start position ..."
    )
    movegroup_interface.go_to_joint_state([1.59, -2.4342, 2.4533, -1.6695, 4.63, 0.0435])

    input(
        "============ Press `Enter` to go the the activate the gripper ..."
    )
    pub.publish(generate_command_msg('a'))
    input(
        "============ Press `Enter` to go the the close the gripper ..."
    )
    pub.publish(generate_command_msg('c'))
    input(
        "============ Press `Enter` to go the the open the gripper ..."
    )
    pub.publish(generate_command_msg('o'))

    input(
        "============ Press `Enter` to move the robot to left ..."
    )
    current_tcp_pose = movegroup_interface.move_group.get_current_pose(end_effector_link = movegroup_interface.move_group.get_end_effector_link())
    rospy.loginfo(f"TCP pose:\n{current_tcp_pose}")
    
    goal_position.append(current_tcp_pose.pose.position.x-0.10)
    goal_position.append(current_tcp_pose.pose.position.y)
    goal_position.append(current_tcp_pose.pose.position.z)

    goal_orientation.append(current_tcp_pose.pose.orientation.x)
    goal_orientation.append(current_tcp_pose.pose.orientation.y)
    goal_orientation.append(current_tcp_pose.pose.orientation.z)
    goal_orientation.append(current_tcp_pose.pose.orientation.w)
    movegroup_interface.go_to_pose_goal(goal_position, goal_orientation)

    input(
        "============ Press `Enter` to move the robot to right ..."
    )
    current_tcp_pose = movegroup_interface.move_group.get_current_pose(end_effector_link = movegroup_interface.move_group.get_end_effector_link())
    rospy.loginfo(f"TCP pose:\n{current_tcp_pose}")
    
    goal_position = []
    goal_orientation = []
    goal_position.append(current_tcp_pose.pose.position.x+0.10)
    goal_position.append(current_tcp_pose.pose.position.y)
    goal_position.append(current_tcp_pose.pose.position.z)
    goal_orientation.append(current_tcp_pose.pose.orientation.x)
    goal_orientation.append(current_tcp_pose.pose.orientation.y)
    goal_orientation.append(current_tcp_pose.pose.orientation.z)
    goal_orientation.append(current_tcp_pose.pose.orientation.w)
    movegroup_interface.go_to_pose_goal(goal_position, goal_orientation)
    input(
        "============ Press `Enter` to go the the home position ..."
    )
    movegroup_interface.go_to_joint_state([1.59, -3.08, 2.68, -2.69, 4.63, 0.0435])
if __name__ == '__main__':
    main()