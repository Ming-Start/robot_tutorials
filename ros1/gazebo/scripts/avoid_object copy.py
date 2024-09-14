#!/usr/bin/env python3
import os
import rospy
import sys
sys.path.append('/home/hym/Code/catkin_fox/src/robot_tutorials/ros1/gazebo')
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

# 导入自定义消息类型
try:
    from fg_gazebo_example.msg import ObjectInfo
except ImportError as e:
    rospy.logerr("Import Error: {0}".format(e))

# 初始化一个全局的marker id
marker_id = 0

def create_marker(object_info, index):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 设置位置和大小
    marker.pose.position.x = object_info.x
    marker.pose.position.y = object_info.y
    marker.pose.position.z = object_info.z

    # 假设障碍物都是直立的，旋转为0
    marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

    marker.scale.x = object_info.length
    marker.scale.y = object_info.depth
    marker.scale.z = object_info.height

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    marker.id = index  # 每个marker需要一个唯一的id
    marker.lifetime = rospy.Duration(0)  # 永久显示

    return marker

def create_collision_object(object_info):
    collision_object = CollisionObject()
    collision_object.id = object_info.class_name
    collision_object.header.frame_id = "world"

    # 定义物体的基本形状和尺寸
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [object_info.length, object_info.depth, object_info.height]

    # 定义物体的位置
    pose = Pose()
    pose.position.x = object_info.z
    pose.position.y = object_info.y
    pose.position.z = object_info.x

    # 没有旋转，使用默认Quaternion
    pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    return collision_object

def callback(data):
    global marker_id  # 使用全局的marker id

    rospy.loginfo("Received object info:")
    rospy.loginfo("Class: %s", data.class_name)
    rospy.loginfo("Position: X=%.3f, Y=%.3f, Z=%.3f", data.x, data.y, data.z)
    rospy.loginfo("Size: L=%.3f, H=%.3f, D=%.3f", data.length, data.height, data.depth)
    rospy.loginfo("-----------------------")

    # 在rviz中显示bounding box
    marker = create_marker(data, marker_id)
    marker_pub.publish(marker)
    marker_id += 1  # 每次发布后增加marker id

    # 在gazebo中生成碰撞物体
    collision_object = create_collision_object(data)
    collision_object_pub.publish(collision_object)

def listener():
    global marker_pub, collision_object_pub

    rospy.init_node('object_info_listener', anonymous=True)

    # 订阅话题
    rospy.Subscriber("/object_info", ObjectInfo, callback)

    # 发布到rviz的marker话题
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    # 发布到gazebo的collision_object话题
    collision_object_pub = rospy.Publisher("collision_object", CollisionObject, queue_size=10)

    # 保持节点运行，直到被手动关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass