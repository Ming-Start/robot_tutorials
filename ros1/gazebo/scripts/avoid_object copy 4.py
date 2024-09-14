#!/usr/bin/env python3
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import sys

sys.path.append('/home/hym/Code/catkin_fox/src/robot_tutorials/ros1/gazebo')
try:
    from fg_gazebo_example.msg import ObjectInfo
except ImportError as e:
    rospy.logerr("Import Error: {0}".format(e))

# 初始化全局变量
scene = None
objects = {}

def transform_camera_to_base(x_cam, y_cam, z_cam):
    """
    将相机坐标系下的坐标转换为机械臂 base_link 坐标系的坐标
    """
    x_base = x_cam  # 相机的X轴 -> 机械臂的X轴
    y_base = z_cam  # 相机的Z轴 -> 机械臂的Y轴（注意可能需要翻转方向）
    z_base = y_cam  # 相机的Y轴 -> 机械臂的Z轴

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
    pose.position.z = obj_info['position'][2]
    pose.orientation.w = 1.0  # 默认朝向

    # 将形状和位置赋给 CollisionObject
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    # 将 CollisionObject 添加到场景中
    scene.add_object(collision_object)

def update_scene():
    global scene, objects

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

def listener():
    global scene
    rospy.init_node('object_info_listener', anonymous=True)

    # 初始化 MoveIt! 的 RobotCommander 和 PlanningSceneInterface
    RobotCommander()
    scene = PlanningSceneInterface()

    # 订阅 /object_info topic
    rospy.Subscriber("/object_info", ObjectInfo, callback)

    # 以5Hz的频率更新场景
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        update_scene()
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass