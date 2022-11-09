#!/usr/bin/python3

# from __future__ import print_function
# from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

from math import pi, tau, dist, fabs, cos


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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

class MissionActionServer(object):
    """MissionActionServer"""

    def __init__(self):
        super(MissionActionServer, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        # self.joint_state_topic = ['joint_states:=/my_gen3/joint_states']
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("mission_as", anonymous=True)

        self.degrees_of_freedom = rospy.get_param(
               "my_gen3" + "/degrees_of_freedom", 6)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander("my_gen3" + "/robot_description")

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        # print(f"apple pie {"my_gen3"}")
        move_group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="my_gen3",
                                                                 name=group_name)

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface(
                ns="my_gen3")

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "my_gen3" + "/move_group/display_planned_path",
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

    def reach_named_position(self, target):
        """ This function makes the robot reach a named position defined in the 
        moveit wizard"""

        arm_group = self.move_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        arm_group.set_max_velocity_scaling_factor(1.0)
        arm_group.set_max_acceleration_scaling_factor(1.0)
        # Plan the trajectory
        (success_flag, trajectory_message,
         planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    def go_to_pose_goal(self):
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
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

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

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        move_group.set_max_velocity_scaling_factor(0.03)
        move_group.set_max_acceleration_scaling_factor(0.01)
        move_group.set_num_planning_attempts(10)

        waypoints = []

        mtarget = moveit_commander.Pose()
        mtarget.position.x = 0.2596238372665062
        mtarget.position.y = 0.5877850010083194
        mtarget.position.z = 1.0238443039530603
        mtarget.orientation.x = -0.5055357895603413
        mtarget.orientation.y = 0.4986347640062258
        mtarget.orientation.z = 0.5102174540797526
        mtarget.orientation.w = 0.48525775331305615

        waypoints.append(mtarget)

        print(waypoints)

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the Mission Action Server")
        print("----------------------------------------------------------")

        tutorial = MissionActionServer()
        tutorial.move_group.set_goal_tolerance(0.005)  # 1/2 cm
        tutorial.move_group.set_planner_id("PRMkConfigDefault")
        # tutorial.move_group.set_max_velocity_scaling_factor(0.3)
        # tutorial.move_group.set_max_acceleration_scaling_factor(0.1)
        # tutorial.move_group.set_planning_time(1)

        # tutorial.reach_named_position("home")

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        # tutorial.go_to_joint_state()

        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # tutorial.go_to_pose_goal()

        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        tutorial.display_trajectory(cartesian_plan)

        # input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)

        # print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
