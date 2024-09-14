#!/usr/bin/env python3
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg


def main():
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('moveit_script', anonymous=True)

    robot = moveit_commander.RobotCommander()
    group_name = robot.get_group_names()[0]
    move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = moveit_commander.PlanningSceneInterface()

    # Add the ground plane as collision object
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    scene.add_plane("ground_plane", plane_pose)

    move_group.set_max_acceleration_scaling_factor(1)
    move_group.set_max_velocity_scaling_factor(1)

    target_joint_states = [
        [0, -math.pi / 2, 0, -math.pi / 2, -math.pi / 2, 0],
        [2.261, -1.088, 0.997, -1.732, -2.251, 0.601],
        [1.207, -1.731, 1.205, -1.642, -1.355, -0.295],
        # [2.261, -1.088, 0.997, -1.732, -2.251, 0.601],
        # [0.628, -1.073, 1.032, -1.471, -0.870, -1.136],
        # [1.458, -1.341, 2.492, -3.641, -1.558, -0.044],
        # [1.458, -0.720, 0.531, -1.635, -1.559, -0.043],
    ]

    for target_state in target_joint_states:
        success = False
        move_group.set_joint_value_target(target_state)
        success, trajectory, planning_time, error_code = move_group.plan()
        move_group.execute(trajectory, wait=True)

    # for target_state in target_joint_states:
    #     move_group.set_joint_value_target(target_state)
    #     success, trajectory, planning_time, error_code = move_group.plan()
    #     if success:
    #         move_group.execute(trajectory, wait=True)
    #         # Print current pose and joint state
    #         current_pose = move_group.get_current_pose().pose
    #         current_joints = move_group.get_current_joint_values()
    #         rospy.loginfo(f"Target Joint State: {target_state}")
    #         rospy.loginfo(f"Current Pose: {current_pose}")
    #         rospy.loginfo(f"Current Joint State: {current_joints}")
    #     else:
    #         rospy.logwarn("Planning failed.")

if __name__ == "__main__":
    main()
