
# 四旋翼飞行器轨迹规划

该项目用于生成并执行四旋翼飞行器在带有障碍物的三维环境中的多项式轨迹。轨迹生成包含A*路径搜索、多项式优化和重规划，以确保飞行器能够安全地穿越环境。该项目基于ROS（机器人操作系统）实现，并使用了多个库进行可视化和数学计算。

## 功能

- **A*路径规划**：从起点到目标点计算一条初步的可行路径，避开障碍物。
- **多项式优化**：使用多项式轨迹优化（Minisnap）来平滑路径并确保平滑过渡。
- **重规划**：检查轨迹中的不安全段，并在必要时重新优化路径。
- **可视化**：在RViz中可视化路径和轨迹。

## 依赖项

确保您安装了以下依赖项：

- **ROS Noetic**（或兼容的ROS版本）
- **Eigen**：用于线性代数运算
- **PCL**（点云库）：用于处理点云
- **OSQP**：一个二次规划求解器，与OsqpEigen一起使用
- **OsqpEigen**：OSQP的C++封装

您可以通过以下命令安装OSQP和OsqpEigen：

```bash
sudo apt-get install ros-noetic-osqp
git clone --recursive https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make
sudo make install
```

## ROS包依赖

确保安装以下ROS包：

```bash
sudo apt install ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-pcl-ros ros-noetic-visualization-msgs 

## 快速开始

### 1. 克隆仓库

```bash
cd ~/catkin_ws/src
git clone <repository_url>
cd ..
catkin_make
source devel/setup.bash
```

### 2. 启动节点

运行轨迹规划节点：

```bash
roslaunch  roslaunch trajectory_generator demo.launch 
```

该启动文件应初始化节点并加载所需参数。您可以根据您的环境设置修改启动文件。

### 参数（从启动文件中读取）

- **轨迹规划**：
  - `planning/vel`：四旋翼的最大速度（默认：1.0）
  - `planning/acc`：四旋翼的最大加速度（默认：1.0）
  - `planning/dev_order`：轨迹优化的导数阶数（默认：3）
  - `planning/min_order`：轨迹优化的最低阶数（默认：3）
- **可视化**：
  - `vis/vis_traj_width`：RViz中轨迹可视化的线条宽度
- **地图**：
  - `map/resolution`：栅格地图的分辨率（默认：0.2）
  - `map/x_size`、`map/y_size`、`map/z_size`：环境的尺寸
- **路径分辨率**：
  - `path/resolution`：简化路径的分辨率（默认：0.05）
- **重规划**：
  - `replanning/thresh_replan`：触发重规划的距离阈值
  - `replanning/thresh_no_replan`：不触发重规划的距离阈值

## 节点结构和回调函数

### 主节点

主节点 (`traj_node`) 订阅和发布以下主题：

- **订阅**：
  - `/odom`（类型：`nav_msgs/Odometry`）：接收四旋翼的位姿和速度信息
  - `/local_pointcloud`（类型：`sensor_msgs/PointCloud2`）：接收表示环境中障碍物的点云
  - `/waypoints`（类型：`nav_msgs/Path`）：接收四旋翼的目标点位

- **发布**：
  - `trajectory`（类型：`quadrotor_msgs/PolynomialTrajectory`）：发布优化的多项式轨迹
  - `vis_trajectory`（类型：`visualization_msgs/Marker`）：发布轨迹的可视化标记，用于在RViz中显示
  - `vis_path`（类型：`visualization_msgs/Marker`）：发布规划路径的可视化标记，用于在RViz中显示

### 核心函数

- **路径规划与简化**：
  - `trajGeneration()`：轨迹生成的主函数，使用A*算法进行初始路径规划，使用RDP简化路径，然后进行多项式优化。
  - `trajOptimization()`：对简化的路径进行Minisnap优化，生成平滑的轨迹。

- **轨迹执行与重规划**：
  - `execCallback()`：以100Hz的频率运行，用于管理四旋翼的状态，并在必要时触发重规划。
  - `rcvOdomCallback()`：接收里程计数据，更新四旋翼的当前位置和速度。
  - `rcvWaypointsCallback()`：接收路径点，更新四旋翼的目标点。
  - `rcvPointCloudCallBack()`：接收点云数据并在栅格地图中设置障碍物，用于路径规划。

- **可视化**：
  - `visPath()`：在RViz中发布路径的可视化标记。
  - `visTrajectory()`：在RViz中发布优化后的轨迹的可视化标记。

### 状态机

节点使用了状态机来管理轨迹生成与执行的流程，状态包括：

- **INIT**：初始化状态，等待里程计和目标点数据。
- **WAIT_TARGET**：等待目标点的状态。
- **GEN_NEW_TRAJ**：生成新轨迹。
- **EXEC_TRAJ**：执行当前轨迹。
- **REPLAN_TRAJ**：根据需要重新规划轨迹。

## 使用示例

1. **设置目标点**：通过发布到 `/waypoints` 主题来设置目标点。
2. **查看生成的轨迹**：在RViz中查看生成的路径和多项式轨迹的可视化。
3. **监控状态转换**：在控制台中查看状态机的状态转换。

## 注意事项

- 代码包含一个安全检查机制，用于验证轨迹的安全性，在必要时执行重规划。
- 障碍物地图根据从 `/local_pointcloud` 主题接收到的点云数据动态更新。
- 可以通过启动文件自定义参数，以适应不同的环境和轨迹需求。

## 许可证

该项目基于MIT许可证开源。

## 参考文献

轨迹规划算法参考自以下论文：
- **"Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environment"**
