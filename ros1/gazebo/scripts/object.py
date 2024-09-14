#!/usr/bin python3
import os
import rospy
import sys
sys.path.append('/home/hym/Code/catkin_fox/src/robot_tutorials/ros1/gazebo')
# 导入自定义消息类型
try:
    from fg_gazebo_example.msg import ObjectInfo
except ImportError as e:
    rospy.logerr("Import Error: {0}".format(e))
    
def callback(data):
    rospy.loginfo("Received object info:")
    rospy.loginfo("Class: %s", data.class_name)
    rospy.loginfo("Position: X=%.3f, Y=%.3f, Z=%.3f", data.x, data.y, data.z)
    rospy.loginfo("Size: L=%.3f, H=%.3f, D=%.3f", data.length, data.height, data.depth)
    rospy.loginfo("-----------------------")

def listener():
    rospy.init_node('object_info_listener', anonymous=True)

    rospy.Subscriber("/object_info", ObjectInfo, callback)

    # 保持节点运行，直到被手动关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass