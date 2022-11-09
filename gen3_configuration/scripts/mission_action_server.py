#!/usr/bin/python3

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
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("mission_as", )

        try:
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("my_gen3/robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.move_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print (e)
        else:
            print("successfully initialized")

    def get_cartesian_pose(self):
        arm_group = self.move_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose
    
    def reach_joint_angles(self, tolerance, joint_positions=[]):
        arm_group = self.move_group
        success = True

        # Get the current joint positions
        # joint_positions = arm_group.get_current_joint_values()
        # rospy.loginfo("Printing current joint positions before movement :")
        # for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        # joint_positions[0] = 0
        # joint_positions[1] = 0
        # joint_positions[2] = pi/2
        # joint_positions[3] = pi/4
        # joint_positions[4] = 0
        # joint_positions[5] = pi/2
        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        # new_joint_positions = arm_group.get_current_joint_values()
        # rospy.loginfo("Printing current joint positions after movement :")
        # for p in new_joint_positions: rospy.loginfo(p)
        return success
    
    def reach_named_position(self, target):
        """ This function makes the robot reach a named position defined in the 
        moveit wizard"""

        arm_group = self.move_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        print(f"All possible named target values: {arm_group.get_named_targets()}")
        arm_group.set_named_target(target)
        # arm_group.set_max_velocity_scaling_factor(1.0)
        # arm_group.set_max_acceleration_scaling_factor(1.0)
        # Plan the trajectory
        (success_flag, trajectory_message,
         planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    def go_to_pose_goal(self):
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        # pose_goal = geometry_msgs.msg.PoseStamped()
        # pose_goal.header.frame_id = "world"
        # pose_goal.pose.position.x = 0.2596238372665062
        # pose_goal.pose.position.y = 0.5877850010083194
        # pose_goal.pose.position.z = 1.0238443039530603
        # pose_goal.pose.orientation.x = -0.5055357895603413
        # pose_goal.pose.orientation.y = 0.4986347640062258
        # pose_goal.pose.orientation.z = 0.5102174540797526
        # pose_goal.pose.orientation.w = 0.48525775331305615

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.move_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        # move_group.set_max_velocity_scaling_factor(0.03)
        # move_group.set_max_acceleration_scaling_factor(0.01)
        move_group.set_num_planning_attempts(10)

        waypoints = []

        mtarget = moveit_commander.Pose()
        mtarget.position.x = 0.43822136
        mtarget.position.y = 0.089
        mtarget.position.z = 0.8036750679792021
        mtarget.orientation.x = 0.0006745600176505448
        mtarget.orientation.y = 0.7134677370703144
        mtarget.orientation.z = -0.0058675983477976265
        mtarget.orientation.w = 0.7006631889989489

        waypoints.append(mtarget)

        print(waypoints)

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def main(self):
        try:
            print("")
            print("----------------------------------------------------------")
            print("Welcome to the Mission Action Server")
            print("----------------------------------------------------------")

            
            self.move_group.set_goal_tolerance(0.005)  # 1/2 cm
            # self.move_group.set_planner_id("OMPLPlanner")
            # self.move_group.set_planning_time(10)
            # print(f"current planner id: {self.move_group.get_planner_id()}")
            # self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            self.move_group.set_max_velocity_scaling_factor(1.0)
            self.move_group.set_max_acceleration_scaling_factor(0.6)
            # self.move_group.set_planning_time(1)

            self.reach_named_position("retract")
            # self.reach_joint_angles()
            # self.go_to_pose_goal()
            cartesian_plan, fraction = self.plan_cartesian_path()
            self.display_trajectory(cartesian_plan)

            # for point in cartesian_plan.joint_trajectory.points:
            #     self.reach_joint_angles(0.005, point.positions)
            # print(cartesian_plan.joint_trajectory.points)
            self.execute_plan(cartesian_plan)


            # actual_pose = self.get_cartesian_pose()
            # actual_pose.position.z -= 0.1
            # # actual_pose.orientation.y += 0.2
            # self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    bot = MissionActionServer()
    bot.main()
