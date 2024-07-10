#!/usr/bin/env python3
import sys
import argparse
import rospy
import moveit_commander
import geometry_msgs.msg

def move_to_position(x, y, z):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = robot.get_group_names()[0]
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Add the ground plane as collision object
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    scene.add_plane("ground_plane", plane_pose)

    move_group.set_max_acceleration_scaling_factor(1)
    move_group.set_max_velocity_scaling_factor(1)
    move_group.set_planning_time(20)  # Increase planning time
    move_group.set_num_planning_attempts(10)  # Increase number of planning attempts

    # Define the target pose in Cartesian space
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0  # Assuming no orientation change needed
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    # Set the target pose
    move_group.set_pose_target(pose_target)

    # Plan and execute the motion
    plan = move_group.go(wait=True)

    # Ensure that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()

    if not plan:
        rospy.logerr(f"Motion planning failed for position ({x}, {y}, {z}). Unable to reach the target position.")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Move UR5 to a specified position.')
    parser.add_argument('x', type=float, help='Target X position')
    parser.add_argument('y', type=float, help='Target Y position')
    parser.add_argument('z', type=float, help='Target Z position')

    args = parser.parse_args()

    try:
        move_to_position(args.x, args.y, args.z)
    except rospy.ROSInterruptException:
        pass
