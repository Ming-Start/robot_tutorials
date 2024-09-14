import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander

def add_mesh_object(scene, name, pose, file_path, scale=(1, 1, 1)):
    scene.add_mesh(name, pose, file_path, scale)
    
def add_box(scene, name, pose, size):
    scene.add_box(name, pose, size)

def add_cylinder(scene, name, pose, height, radius):
    scene.add_cylinder(name, pose, height, radius)

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

    # 添加桌面
    table_top_name = "table_top"
    table_top_pose = geometry_msgs.msg.PoseStamped()
    table_top_pose.header.frame_id = "world"
    table_top_pose.pose.position.x = 0
    table_top_pose.pose.position.y = 1
    table_top_pose.pose.position.z = 0.25
    table_top_pose.pose.orientation.w = 1.0
    table_top_size = (1.5, 0.8, 0.05)
    add_box(scene, table_top_name, table_top_pose, table_top_size)

    # 添加碗
    bowl_name_0 = "bowl_0"
    bowl_pose_0 = geometry_msgs.msg.PoseStamped()
    bowl_pose_0.header.frame_id = "world"
    bowl_pose_0.pose.position.x = -0.3
    bowl_pose_0.pose.position.y = 0.7
    bowl_pose_0.pose.position.z = 0.27
    bowl_pose_0.pose.orientation.w = 1.0
    bowl_file_path = "/usr/share/gazebo-11/models/Footed_Bowl_Sand/meshes/model.obj"
    add_mesh_object(scene, bowl_name_0, bowl_pose_0, bowl_file_path)

    bowl_name_1 = "bowl_1"
    bowl_pose_1 = geometry_msgs.msg.PoseStamped()
    bowl_pose_1.header.frame_id = "world"
    bowl_pose_1.pose.position.x = 0.3
    bowl_pose_1.pose.position.y = 0.7
    bowl_pose_1.pose.position.z = 0.27
    bowl_pose_1.pose.orientation.w = 1.0
    add_mesh_object(scene, bowl_name_1, bowl_pose_1, bowl_file_path)

    # 添加瓶子
    beer_name = "bottle"
    beer_pose = geometry_msgs.msg.PoseStamped()
    beer_pose.header.frame_id = "world"
    beer_pose.pose.position.x = 0.0
    beer_pose.pose.position.y = 0.7
    beer_pose.pose.position.z = 0.27
    beer_pose.pose.orientation.w = 1.0
    beer_file_path = "/usr/share/gazebo-11/models/Weston_No_22_Cajun_Jerky_Tonic_12_fl_oz_nLj64ZnGwDh/meshes/model.obj"
    add_mesh_object(scene, beer_name, beer_pose, beer_file_path)

    # 等待场景物体添加完成
    rospy.sleep(2)

    # 设置规划器
    move_group.set_planner_id("RRTConnectkConfigDefault")

    # 设置规划参数
    move_group.set_planning_time(30.0)
    move_group.set_num_planning_attempts(50)

    # 设置避障参数
    move_group.set_planning_time(20.0)
    move_group.set_num_planning_attempts(20)
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.01)

    # 设置目标末端点位置
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.4
    target_pose.position.y = 0.1
    target_pose.position.z = 0.4
    target_pose.orientation.w = 1.0

    # 设置目标位置并进行规划
    move_group.set_pose_target(target_pose)

    # 规划并执行路径
    plan = move_group.go(wait=True)

    # 停止运动
    move_group.stop()
    move_group.clear_pose_targets()

    # 检查规划结果
    if plan:
        rospy.loginfo("Path planning succeeded.")
    else:
        rospy.logerr("Path planning failed.")

if __name__ == "__main__":
    main()
    
