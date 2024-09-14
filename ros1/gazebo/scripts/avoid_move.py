#!/usr/bin python3
import sys
import math
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped

def update_scene(scene):
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    # 获取障碍物状态
    obstacle_name = 'cubic_obstacle'
    obstacle_state = get_model_state(obstacle_name, '')

    # 更新MoveIt!的规划场景
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose = obstacle_state.pose
    scene.add_box(obstacle_name, box_pose, size=(0.2, 0.2, 0.2))

def move_arm():
    rospy.init_node('move_arm', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander("manipulator")
    scene = PlanningSceneInterface()

    # 更新规划场景
    update_scene(scene)

    # 设置初始目标关节状态
    initial_joint_states = [0, 0, 0, 0, 0]
    arm.set_joint_value_target(initial_joint_states)
    success, trajectory, planning_time, error_code = arm.plan()
    if success:
        arm.execute(trajectory, wait=True)
        rospy.loginfo("Initial joint state achieved.")
    else:
        rospy.logwarn("Planning to initial joint state failed.")
        return

    # 设置目标位置
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.pose.orientation.w = 1.0
    target_pose.pose.position.x = 0.85
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.34
    arm.set_pose_target(target_pose.pose)

    # 设置规划时间和规划器
    # arm.set_planning_time(100)
    # arm.set_planner_id("RRTConnectkConfigDefault")

    # 规划并执行
    plan = arm.go(wait=True)
    if plan:
        rospy.loginfo("Target position reached.")
    else:
        rospy.logwarn("Failed to reach the target position.")

    arm.stop()
    arm.clear_pose_targets()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException:
        pass