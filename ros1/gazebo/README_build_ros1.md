# Robot Tutorial Project  
  
## 编译并运行 Robot Tutorial Project  
  
### 第一步：创建catkin_ws包并转入到src目录下  
  
1. 打开你的终端或命令行界面。  
2. 创建一个名为`catkin_ws`的工作空间，并在其内部创建一个`src`目录用于存放项目源代码。  
  
   ```bash  
   mkdir -p catkin_ws/src  
   cd catkin_ws/src

### 第二步：从所需要的branch中git clone代码  
  
1. **打开命令行界面**：首先，打开你的终端（在Linux或macOS上）或命令提示符/PowerShell（在Windows上）。  
  
2. **导航到`catkin_ws/src`目录**：如果你还没有按照第一步的指示创建并导航到这个目录，请现在这样做。如果已经在该目录下，可以跳过这一步。  
  
3. **执行`git clone`命令**：在`src`目录下，执行以下命令来克隆`robot_tutorials`项目。请注意，如果你需要克隆特定的分支（而不是默认的`main`或`master`分支），你需要在命令中指定分支名（替换下面的`your-branch-name`）。  
  
   ```bash  
   git clone -b branch-name https://github.com/Ming-Start/robot_tutorials.git

### 第三步： 安装依赖项

1. **在ros2文件夹中添加CATKIN_IGNORE文件**:为了避免`catkin`构建系统处理`ros2`相关的包（项目中混合了`ROS1`和`ROS2`的包），在`robot_tutorials/ros2`目录下创建一个名为`CATKIN_IGNORE`的空文件。

   ```bash
   touch src/robot_tutorials/ros2/CATKIN_IGNORE

2. **在catkin_ws文件夹中安装依赖项**:使用`rosdep`工具安装`robot_tutorials`项目所需的依赖项。

   ```bash
   rosdep install -i --from-path src

### 第四步： 在`catkin_ws`文件夹目录下执行构建并source

1. **编译`fg_gazebo_example`**:返回到`catkin_ws`目录，并使用`catkin build`命令构建项目。注意，这里要构建的是fg_gazebo_example包。

   ```bash
   catkin build fg_gazebo_example

2. **执行source**:为了使`rosrun`、`roslaunch`等ROS命令能够找到你的工作空间中的包，需要source你的工作空间设置脚本。

   ```bash
   source ./devel/setup.bash

### 第五步： 启动仿真

1. **启动`fg_gazebo_example`仿真**:使用`roslaunch`命令启动`fg_gazebo_example`包的仿真。

   ```bash
   roslaunch fg_gazebo_example simulation.launch
