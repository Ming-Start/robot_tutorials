#!/usr/bin/env python3
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander

def add_box(scene, box_name, box_pose, box_size):
    scene.add_box(box_name, box_pose, box_size)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_script', anonymous=True)

    robot = RobotCommander()
    group_name = robot.get_group_names()[0]
    move_group = MoveGroupCommander(group_name)
    scene = PlanningSceneInterface()

    # 添加地面平面
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    scene.add_plane("ground_plane", plane_pose)
  
    # 在仿真环境中添加一个正方体障碍物
    box_name = "obstacle_box"
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0.5  # 在机械臂正前方
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.35  # 正方体中心高度
    box_size = (0.2, 0.2, 0.2)
    add_box(scene, box_name, box_pose, box_size)
    rospy.sleep(2)  # 等待场景更新

    move_group.set_max_acceleration_scaling_factor(1)
    move_group.set_max_velocity_scaling_factor(1)

    # 设置初始目标关节状态
    initial_joint_states = [0, -math.pi / 2, 0, -math.pi / 2, -math.pi / 2, 0]
    move_group.set_joint_value_target(initial_joint_states)
    success, trajectory, planning_time, error_code = move_group.plan()
    if success:
        move_group.execute(trajectory, wait=True)
        rospy.loginfo("Initial joint state achieved.")
    else:
        rospy.logwarn("Planning to initial joint state failed.")

    # 设置从上方移动到下方的目标关节状态
    target_joint_states = [
        [0, -math.radians(45), 0, -math.radians(90), -math.radians(90), -math.radians(90)],  # 上方
        [0, 0, 0, -math.radians(90), -math.radians(90), -math.radians(90)]  # 下方
    ]

    for target_state in target_joint_states:
        move_group.set_joint_value_target(target_state)
        success, trajectory, planning_time, error_code = move_group.plan()
        if success:
            move_group.execute(trajectory, wait=True)
            rospy.loginfo(f"Moved to target state: {target_state}")
        else:
            rospy.logwarn(f"Planning to target state {target_state} failed.")

if __name__ == "__main__":
    main()
