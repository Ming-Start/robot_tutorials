#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from shapely.geometry import box, Point as ShapelyPoint
import numpy as np

sys.path.append('/home/hym/Code/catkin_fox/src/robot_tutorials/ros1/gazebo')
try:
    from fg_gazebo_example.msg import ObjectInfo
except ImportError as e:
    rospy.logerr("Import Error: {0}".format(e))

# 初始化全局变量
scene = None
obstacles = {}

def transform_camera_to_base(x_cam, y_cam, z_cam):
    """
    将相机坐标系下的坐标转换为机械臂 base_link 坐标系的坐标
    """
    x_base = x_cam  # 相机的X轴 -> 机械臂的X轴
    y_base = z_cam  # 相机的Z轴 -> 机械臂的Y轴（注意可能需要翻转方向）
    z_base = y_cam   # 相机的Y轴 -> 机械臂的Z轴

    return x_base, y_base, z_base

def callback(data):
    global obstacles
    # 将相机坐标系下的物体位置转换到机械臂 base_link 坐标系
    x_base, y_base, z_base = transform_camera_to_base(data.x, data.y, data.z)

    # 更新或添加物体信息到字典中
    obstacles[data.class_name] = {
        'length': data.length,
        'depth': data.depth,
        'height': data.height,
        'position': (x_base, y_base, z_base)
    }

def update_scene():
    global scene, obstacles

    # 清除所有物体
    for obj_id in obstacles.keys():
        scene.remove_world_object(obj_id)

    # 重新添加所有物体
    updated_obstacles = []
    for obj_id, obj_info in obstacles.items():
        collision_object = CollisionObject()
        collision_object.id = obj_id
        collision_object.header.frame_id = "base_link"  # 使用机械臂的 base_link 坐标系

        # 定义物体的形状和尺寸
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [obj_info['length'], obj_info['depth'], obj_info['height']]

        # 定义物体的位置
        pose = Pose()
        pose.position.x = obj_info['position'][0]
        pose.position.y = obj_info['position'][1]
        pose.position.z = obj_info['position'][2]
        pose.orientation.w = 1.0  # 默认朝向

        # 将形状和位置赋给 CollisionObject
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        # 将 CollisionObject 添加到场景中
        scene.add_object(collision_object)

        # 将障碍物信息添加到更新的列表中
        updated_obstacles.append((pose, [obj_info['length'], obj_info['depth'], obj_info['height']]))

    rospy.loginfo("Updated scene with current objects.")
    return updated_obstacles

def is_target_in_obstacle(target_position, obstacles):
    for obstacle_pose, obstacle_size in obstacles:
        obs_min_x = obstacle_pose.position.x - obstacle_size[0] / 2
        obs_max_x = obstacle_pose.position.x + obstacle_size[0] / 2
        obs_min_y = obstacle_pose.position.y - obstacle_size[1] / 2
        obs_max_y = obstacle_pose.position.y + obstacle_size[1] / 2
        obs_min_z = obstacle_pose.position.z - obstacle_size[2] / 2
        obs_max_z = obstacle_pose.position.z + obstacle_size[2] / 2

        obstacle_box = box(obs_min_x, obs_min_y, obs_max_x, obs_max_y)
        target_point = ShapelyPoint(target_position[0], target_position[1])

        if obstacle_box.contains(target_point):
            if obs_min_z <= target_position[2] <= obs_max_z:
                return True
    return False

def is_in_collision(scene, robot, move_group):
    current_state = robot.get_current_state()
    move_group.set_start_state(current_state)
    
    waypoints = []
    waypoints.append(move_group.get_current_pose().pose)
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0)

    return fraction < 1.0

def move_to_safe_position(move_group):
    safe_joint_position = [0, -1.57, 0, -1.57, 0, 0]

    move_group.set_joint_value_target(safe_joint_position)
    
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def move_backwards(move_group, distance):
    current_pose = move_group.get_current_pose().pose
    current_pose.position.z -= distance
    move_group.set_pose_target(current_pose)
    move_group.go(wait=True)
    move_group.stop()

def try_planning_with_different_planners(move_group, planners, target_position, scene, robot, max_attempts=3):
    move_group.set_planning_time(10)

    best_plan = None
    for attempt in range(max_attempts):
        for planner_id in planners:
            rospy.loginfo(f"Attempt {attempt + 1}/{max_attempts} - Trying planner: {planner_id}")
            move_group.set_planner_id(planner_id)
            
            target_pose = move_group.get_current_pose().pose
            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]
            move_group.set_pose_target(target_pose)

            success, plan, _, _ = move_group.plan()

            if success:
                rospy.loginfo("Checking for collisions before executing plan...")
                if is_in_collision(scene, robot, move_group):
                    rospy.logwarn("Pre-execution collision detected! Aborting and trying next planner.")
                    continue

                if best_plan is None or len(plan.joint_trajectory.points) < len(best_plan.joint_trajectory.points):
                    best_plan = plan

        if best_plan:
            move_group.execute(best_plan, wait=False)

            for _ in range(50):  # 动态检查碰撞
                updated_obstacles = update_scene()
                if is_in_collision(scene, robot, move_group):
                    rospy.logwarn("Collision detected during execution! Stopping the robot...")
                    
                    move_group.stop()
                    move_group.clear_pose_targets()
                    
                    move_backwards(move_group, distance=0.1)
                    
                    return try_planning_with_different_planners(move_group, planners, target_position, scene, robot, max_attempts)
                
                rospy.sleep(0.2)

            rospy.loginfo(f"Successfully moved to position using {planner_id}.")
            return True
        else:
            rospy.logwarn(f"All planners failed on attempt {attempt + 1}. Moving to safe position and retrying...")
            move_to_safe_position(move_group)

    rospy.logerr("All planning attempts failed. Unable to reach the target position.")
    return False

def move_to_position(x, y, z, move_to_safe=True):
    global scene
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("manipulator")
    scene = PlanningSceneInterface()
    robot = RobotCommander()

    if move_to_safe:
        move_to_safe_position(move_group)

    # 订阅 /object_info topic，获取障碍物信息
    rospy.Subscriber("/object_info", ObjectInfo, callback)

    rospy.sleep(2)  # 等待初始障碍物信息的获取

    target_position = [x, y, z]

    # 更新场景并获取所有障碍物的信息
    obstacles = update_scene()

    # 检查目标点是否在任何一个障碍物内
    if is_target_in_obstacle(target_position, obstacles):
        rospy.logerr(f"Target position ({x}, {y}, {z}) is inside an obstacle. Please choose a different target.")
        return

    planners = [
        "RRTConnectkConfigDefault",
        "RRTstar",
        "PRMkConfigDefault"
    ]

    success = try_planning_with_different_planners(move_group, planners, target_position, scene, robot)

    if success:
        rospy.loginfo(f"Successfully moved to position ({x}, {y}, {z}) while avoiding obstacles.")
    else:
        rospy.logerr(f"Failed to move to position ({x}, {y}, {z}) while avoiding obstacles.")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    # 第一次运行时，先移动到安全位置
    move_to_position(0.1, 0.7, 0.3, move_to_safe=True)