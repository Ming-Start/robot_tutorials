#!/usr/bin/env python3
import rospy
import sys
import threading
from threading import Event
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped
from shapely.geometry import box, Point
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# 导入自定义消息类型
try:
    from fg_gazebo_example.msg import ObjectInfo
except ImportError as e:
    rospy.logerr("Import Error: {0}".format(e))

# 初始化全局变量
scene = None
objects = {}
scene_update_event = Event()
scene_updated = Event()

def transform_camera_to_base(x_cam, y_cam, z_cam):
    """
    将相机坐标系下的坐标转换为机械臂 base_link 坐标系的坐标
    """
    x_base = x_cam
    y_base = z_cam  # 根据实际情况调整坐标系
    z_base = y_cam
    return x_base, y_base, z_base

def callback(data):
    global objects
    # 将相机坐标系下的物体位置转换到机械臂 base_link 坐标系
    x_base, y_base, z_base = transform_camera_to_base(data.x, data.y, data.z)

    # 更新或添加物体信息到字典中
    objects[data.class_name] = {
        'length': data.length,
        'depth': data.depth,
        'height': data.height,
        'position': (x_base, y_base, z_base)
    }

    # 触发场景更新事件，让定时器进行场景更新
    scene_update_event.set()

def add_object_to_scene(obj_id, obj_info):
    """
    向场景中添加物体
    """
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
    pose.position.z = obj_info['position'][2] + 0.1
    pose.orientation.w = 1.0  # 默认朝向

    # 将形状和位置赋给 CollisionObject
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    # 将 CollisionObject 添加到场景中
    scene.add_object(collision_object)

def add_ground_plane():
    """
    向场景中添加地面平面
    """
    ground = CollisionObject()
    ground.id = "ground_plane"
    ground.header.frame_id = "base_link"
    
    # 定义地面对象的形状和尺寸
    ground_primitive = SolidPrimitive()
    ground_primitive.type = SolidPrimitive.BOX
    ground_primitive.dimensions = [10, 10, 0.1]  # 一个宽度和深度为10米，高度为0.1米的平面

    # 定义地面对象的位置
    ground_pose = Pose()
    ground_pose.position.x = 0
    ground_pose.position.y = 0
    ground_pose.position.z = -0.05  # 假设地面在 base_link 的下方0.05米处
    ground_pose.orientation.w = 1.0  # 默认朝向

    # 将形状和位置赋给 CollisionObject
    ground.primitives.append(ground_primitive)
    ground.primitive_poses.append(ground_pose)
    ground.operation = CollisionObject.ADD

    # 将地面平面添加到场景中
    scene.add_object(ground)

def update_scene(event):
    global scene, objects

    if not scene_update_event.is_set():
        return  # 如果没有新的物体信息，不更新场景

    # 添加地面平面
    add_ground_plane()

    # 获取已知物体的名称列表
    known_objects = scene.get_known_object_names()

    for obj_id, obj_info in objects.items():
        if obj_id in known_objects:  # 检查物体是否已经存在
            # 获取场景中的现有物体信息
            existing_objects = scene.get_objects([obj_id])
            existing_object = existing_objects.get(obj_id, None)

            # 检查是否需要更新物体
            if existing_object and object_needs_update(existing_object, obj_info):
                scene.remove_world_object(obj_id)  # 先移除旧的物体
                add_object_to_scene(obj_id, obj_info)  # 然后添加新的物体
        else:
            # 如果物体不存在，则直接添加
            add_object_to_scene(obj_id, obj_info)

    rospy.loginfo("Updated scene with current objects.")
    scene_update_event.clear()  # 重置场景更新事件
    scene_updated.set()  # 触发场景更新完成事件

def object_needs_update(existing_object, new_info):
    """
    判断场景中的物体是否需要更新
    """
    if (existing_object.primitive_poses[0].position.x != new_info['position'][0] or
        existing_object.primitive_poses[0].position.y != new_info['position'][1] or
        existing_object.primitive_poses[0].position.z != new_info['position'][2]):
        return True
    if (existing_object.primitives[0].dimensions != [new_info['length'], new_info['depth'], new_info['height']]):
        return True
    return False

def is_target_in_obstacle(target_position, obstacles):
    for obstacle_info in obstacles:
        obstacle_pose = obstacle_info['position']
        obstacle_size = (obstacle_info['length'], obstacle_info['depth'], obstacle_info['height'])

        obs_min_x = obstacle_pose[0] - obstacle_size[0] / 2
        obs_max_x = obstacle_pose[0] + obstacle_size[0] / 2
        obs_min_y = obstacle_pose[1] - obstacle_size[1] / 2
        obs_max_y = obstacle_pose[1] + obstacle_size[1] / 2
        obs_min_z = obstacle_pose[2] - obstacle_size[2] / 2
        obs_max_z = obstacle_pose[2] + obstacle_size[2] / 2

        obstacle_box = box(obs_min_x, obs_min_y, obs_max_x, obs_max_y)
        target_point = Point(target_position[0], target_position[1])

        if obstacle_box.contains(target_point) and obs_min_z <= target_position[2] <= obs_max_z:
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
        0.0
    )

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

            for _ in range(50):
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
    moveit_commander.roscpp_initialize(sys.argv)

    global scene
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    scene = PlanningSceneInterface()
    robot = RobotCommander()

    if move_to_safe:
        move_to_safe_position(move_group)

    target_position = [x, y, z]

    # 等待场景更新完成
    rospy.loginfo("Waiting for scene to be updated...")
    scene_updated.wait()

    # 检查目标点是否在任何一个障碍物内
    if is_target_in_obstacle(target_position, objects.values()):
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

def listener():
    global scene
    # 订阅 /object_info topic
    rospy.Subscriber("/object_info", ObjectInfo, callback)

if __name__ == '__main__':
    rospy.init_node('move_ur5e', anonymous=True)

    # 创建一个定时器，每隔30秒更新一次场景
    rospy.Timer(rospy.Duration(10.0), update_scene)

    # 启动监听线程
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    # 让机械臂移动到指定的位置
    move_to_position(0.1, 0.7, 0.25)

    # 等待监听线程结束
    listener_thread.join()