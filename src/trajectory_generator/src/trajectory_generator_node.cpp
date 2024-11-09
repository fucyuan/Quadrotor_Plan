#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <OsqpEigen/OsqpEigen.h>

// Useful customized headers
#include "Astar_searcher.h"
#include "backward.hpp"
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint *_trajGene = new TrajectoryGeneratorWaypoint();
AstarPathFinder *_astar_path_finder = new AstarPathFinder();
OsqpEigen::Solver slover;

// Set the obstacle map
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _traj_vis_pub, _traj_pub, _path_vis_pub;

// for planning
Vector3d odom_pt, odom_vel, start_pt, target_pt, start_vel;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;

// for replanning
enum STATE {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ
} exec_state = STATE::INIT;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
void execCallback(const ros::TimerEvent &e);

// declare
void changeState(STATE new_state, string pos_call);
void printState();
void visTrajectory(MatrixXd polyCoeff, VectorXd time);
void visPath(MatrixXd nodes);
void trajOptimization(Eigen::MatrixXd path);
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom);
void rcvWaypointsCallback(const nav_msgs::Path &wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);
void trajPublish(MatrixXd polyCoeff, VectorXd time);
bool trajGeneration();
VectorXd timeAllocation(MatrixXd Path);
Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);

// 状态转移函数，更改当前的状态到新的状态
void changeState(STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

// 打印当前的状态
void printState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

// 获取当前的位置和速度
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  odom_pt(0) = odom->pose.pose.position.x;
  odom_pt(1) = odom->pose.pose.position.y;
  odom_pt(2) = odom->pose.pose.position.z;

  odom_vel(0) = odom->twist.twist.linear.x;
  odom_vel(1) = odom->twist.twist.linear.y;
  odom_vel(2) = odom->twist.twist.linear.z;

  has_odom = true;
}

// 控制轨迹生成器的状态改变，刷新频率为100Hz
void execCallback(const ros::TimerEvent &e) {
  static int num = 0;                       // 计数器
  num++;
  if (num == 100) {
    printState();                           // 每隔100次打印一次当前的状态
    if (!has_odom)
      cout << "no odom." << endl;
    if (!has_target)
      cout << "wait for goal." << endl;
    num = 0;
  }

  // 状态机
  switch (exec_state) {                 
  case INIT: {                          // 如果当前的状态为初始化
    if (!has_odom)                      // 获取到位置和速度信息
      return;
    if (!has_target)                    // 获取到目标点信息(这句应该是多余的，因为在WAIT_TARGET状态里也有这个判断)
      return;
    changeState(WAIT_TARGET, "STATE");  // 更改状态为等待目标点
    break;
  }

  case WAIT_TARGET: {                   // 如果当前状态为等待目标点
    if (!has_target)
      return;
    else
      changeState(GEN_NEW_TRAJ, "STATE");// 更改状态为生成新的轨迹
    break;
  }

  case GEN_NEW_TRAJ: {                    // 如果当前状态为生成新的轨迹
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");    // 如果生成成功，则更新状态为执行轨迹
    else
      changeState(GEN_NEW_TRAJ, "STATE"); // 如果生成失败，则更新状态为继续生成轨迹
    break;
  }

 case EXEC_TRAJ: {  // 如果当前状态为执行轨迹
    // 获取当前的系统时间
    ros::Time time_now = ros::Time::now();

    // 计算从轨迹开始时间到当前时间的间隔（当前轨迹执行的时间）
    double t_cur = (time_now - time_traj_start).toSec();

    // 设置重规划的时间间隔为1秒（每隔1秒重新检查是否需要重规划）
    double t_replan = ros::Duration(1, 0).toSec();

    // 确保t_cur不超过轨迹总时间（time_duration）
    t_cur = min(time_duration, t_cur);

    // 检查当前轨迹是否接近结束，如果接近结束，则认为到达目标并更新状态为等待目标
    if (t_cur > time_duration - 1e-2) {
        has_target = false;  // 取消目标
        changeState(WAIT_TARGET, "STATE");  // 切换状态为等待目标
        return;
    }
    // 检查是否接近目标点，如果距离小于阈值(no_replan_thresh)，则保持当前轨迹执行，不进行重规划
    else if ((target_pt - odom_pt).norm() < no_replan_thresh) {
        return;
    }
    // 检查是否远离起点，如果距离小于阈值(replan_thresh)，则保持当前轨迹执行，不进行重规划
    else if ((start_pt - odom_pt).norm() < replan_thresh) {
        return;
    }
    // 如果当前时间小于重规划时间间隔(t_replan)，则继续执行当前轨迹
    else if (t_cur < t_replan) {
        return;
    }
    // 如果满足以上条件，即时间和距离满足要求，则更新状态为重新规划轨迹
    else {
        changeState(REPLAN_TRAJ, "STATE");  // 切换状态为重新规划轨迹
    }
    break;
}

  case REPLAN_TRAJ: {                                                 // 如果当前状态为重新规划轨迹
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_delta = ros::Duration(0, 10).toSec();
    t_cur = t_delta + t_cur;
    // start_pt = getPos(t_cur);
    // start_vel = getVel(t_cur);
    start_pt = odom_pt;
    start_vel = odom_vel;
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }
  }
}

// 获取路径规划的终点
void rcvWaypointsCallBack(const nav_msgs::Path &wp) {
  if (wp.poses[0].pose.position.z < 0.0)
    return;

  // 设置目标位置
  target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y,
      wp.poses[0].pose.position.z;
  ROS_INFO("[node] receive the planning target");

  // 起点的位置和速度为当前的位置和速度
  start_pt = odom_pt;
  start_vel = odom_vel;
  has_target = true;

  // 判断当前的状态
  if (exec_state == WAIT_TARGET)
    changeState(GEN_NEW_TRAJ, "STATE"); // 如果当前的状态为等待目标点，则改变状态为生成轨迹
  else if (exec_state == EXEC_TRAJ)
    changeState(REPLAN_TRAJ, "STATE");  // 如果当前的状态为正在执行轨迹，则改变状态为重新规划轨迹
}

// 获取当前位置附近的局部点云，并将其用于A*算法的障碍物地图设置
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {

  pcl::PointCloud<pcl::PointXYZ> cloud;      // 定义用于存储点云数据的PCL点云对象
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;  // 定义用于可视化的点云对象
  sensor_msgs::PointCloud2 map_vis;          // 定义用于发布的ROS点云消息对象

  // 将ROS消息格式的点云转换为PCL格式
  pcl::fromROSMsg(pointcloud_map, cloud);

  // 如果点云为空，直接返回
  if ((int)cloud.points.size() == 0)
    return;

  pcl::PointXYZ pt;  // 定义一个单个点的变量
  for (int idx = 0; idx < (int)cloud.points.size(); idx++) {  // 遍历每个点
    pt = cloud.points[idx];  // 获取当前点
    // 将该点设置为路径规划中的障碍物
    _astar_path_finder->setObs(pt.x, pt.y, pt.z);
  }
}



// 轨迹生成
bool trajGeneration() {
  /**
   *
   * STEP 1:  使用Astar算法获取一条可行的路径
   *
   * **/
  _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
  auto grid_path = _astar_path_finder->getPath();
  // 重置地图
  _astar_path_finder->resetUsedGrids();

  /**
   *
   * STEP 2:  使用RDP算法简化轨迹(由于选取的路径分辨率与栅格分辨率是相同的，这里可以认为是另外一种JPS算法)
   *
   * **/
  grid_path = _astar_path_finder->pathSimplify(grid_path, _path_resolution);
  MatrixXd path(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path.row(k) = grid_path[k];
  }
  /**
   *
   * STEP 3:  Minisnap轨迹优化
   *
   * **/
  trajOptimization(path);
  time_duration = _polyTime.sum();

  // 发布轨迹
  trajPublish(_polyCoeff, _polyTime);

  // 记录轨迹开始的时间
  time_traj_start = ros::Time::now();

  // 返回是否轨迹生成成功
  if (_polyCoeff.rows() > 0)
    return true;
  else
    return false;
}

void trajOptimization(Eigen::MatrixXd path) {
  // if( !has_odom ) return;
  MatrixXd vel = MatrixXd::Zero(2, 3);
  MatrixXd acc = MatrixXd::Zero(2, 3);

  vel.row(0) = start_vel;

  /**
   *
   * STEP 3.1:  时间分配
   *
   * **/
  // _polyTime = timeAllocation(path);
  try {
    _polyTime = timeAllocation(path);
} catch (const std::runtime_error& e) {
    std::cerr << "Error in timeAllocation: " << e.what() << std::endl;
    return;
}


  /**
   *
   * STEP 3.2:  生成minisnap轨迹
   *
   * **/
  // Minisnap轨迹优化
  _polyCoeff =
      _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime,slover);

  // check if the trajectory is safe, if not, do reoptimize
  int unsafe_segment;

  /**
   *
   * STEP 3.3:  轨迹安全性检查
   *
   * **/
  unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
  unsafe_segment = -1;
  MatrixXd repath = path;
  // int count = 0;
  // while (unsafe_segment != -1) {
  //   std::cout << unsafe_segment << std::endl;
  //   /**
  //    *
  //    * STEP 3.4:  reoptimize
  //    * the method comes from: "Polynomial Trajectory
  //    * Planning for Aggressive Quadrotor Flight in Dense Indoor Environment"
  //    * part 3.5
  //    *
  //    * **/
  //   // 轨迹重新规划
  //   Eigen::Vector3d start = repath.row(unsafe_segment);
  //   Eigen::Vector3d end = repath.row(unsafe_segment + 1);
  //   _astar_path_finder->AstarGraphSearch(start, end);
  //   vector<Vector3d> grid_path = _astar_path_finder->getPath();
  //   // Eigen::Vector3d mid = (start + end)/2;
  //   // Eigen::MatrixXd tmp(int(repath.rows()+1),3);
  //   // std::cout << mid << std::endl;
  //   // tmp.block(0,0,unsafe_segment + 1,3) = repath.block(0,0,unsafe_segment + 1,3);
  //   // tmp.row(unsafe_segment+1) = mid;

  //   // tmp.block(unsafe_segment + 2,0,int(repath.rows()- unsafe_segment - 1),3) = repath.block(unsafe_segment + 1,0,int(repath.rows()- unsafe_segment - 1),3);

  //   // repath = tmp;

  //   // _polyTime = timeAllocation(repath);
  //   // _polyCoeff =
  //   //   _trajGene->PolyQPGeneration(_dev_order, repath, vel, acc, _polyTime,slover);
  //   // unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
  // }
  while (unsafe_segment != -1) {
    std::cout << "Current unsafe segment: " << unsafe_segment << std::endl;
    
    // 获取不安全段的起点和终点
    Eigen::Vector3d start = repath.row(unsafe_segment);
    Eigen::Vector3d end = repath.row(unsafe_segment + 1);

    // 使用 A* 重新规划该段的路径
    _astar_path_finder->AstarGraphSearch(start, end);
    std::vector<Eigen::Vector3d> grid_path = _astar_path_finder->getPath();

    // 如果 `grid_path` 为空，则说明重新规划失败，处理这种情况
    if (grid_path.empty()) {
        std::cerr << "Failed to replan path for segment " << unsafe_segment << std::endl;
        break;
    }

    // 将 `grid_path` 插入 `repath` 中以替换不安全段
    Eigen::MatrixXd tmp(repath.rows() + grid_path.size() - 2, 3); // 新矩阵大小：原大小 + 新路径大小 - 被替换的2个点

    // 保留原始路径中不安全段之前的点
    tmp.block(0, 0, unsafe_segment, 3) = repath.block(0, 0, unsafe_segment, 3);

    // 插入新的路径段（`grid_path`），不包含起点和终点，因为它们已在原路径中
    for (size_t j = 0; j < grid_path.size(); ++j) {
        tmp.row(unsafe_segment + j) = grid_path[j];
    }

    // 保留原始路径中不安全段之后的点
    tmp.block(unsafe_segment + grid_path.size(), 0, repath.rows() - unsafe_segment - 2, 3) =
        repath.block(unsafe_segment + 2, 0, repath.rows() - unsafe_segment - 2, 3);

    // 更新路径
    repath = tmp;

    // 重新分配时间和生成多项式轨迹
    // _polyTime = timeAllocation(repath);
      try {
      _polyTime = timeAllocation(repath);
  } catch (const std::runtime_error& e) {
      std::cerr << "Error in timeAllocation: " << e.what() << std::endl;
      return;
  }

    _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, repath, vel, acc, _polyTime, slover);

    // 检查是否还有不安全段
    unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
}

  // visulize path and trajectory
  visPath(repath);
  visTrajectory(_polyCoeff, _polyTime);
}

void trajPublish(MatrixXd polyCoeff, VectorXd time) {
  // 如果多项式系数或时间为空，则警告并返回
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to publish.");
    return;
  }

  unsigned int poly_number;  // 存储多项式阶数

  static int count = 1; // 计数器，表示轨迹ID，每个轨迹的ID必须大于0

  // 定义轨迹消息对象
  quadrotor_msgs::PolynomialTrajectory traj_msg;

  // 设置轨迹消息的头部信息
  traj_msg.header.seq = count;                   // 设置序列号
  traj_msg.header.stamp = ros::Time::now();      // 设置时间戳
  traj_msg.header.frame_id = std::string("world"); // 设置坐标系

  // 设置轨迹ID和动作类型
  traj_msg.trajectory_id = count;                  // 轨迹ID
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD; // 动作类型为“添加轨迹”

  // 设置多项式的阶数（根据_dev_order计算）
  traj_msg.num_order = 2 * _dev_order - 1;  // 多项式的最高阶
  traj_msg.num_segment = time.size();       // 设置轨迹段数

  // 计算初始和终止速度方向，确定yaw角
  Vector3d initialVel, finalVel;
  initialVel = _trajGene->getVelPoly(_polyCoeff, 0, 0);  // 初始段的速度
  finalVel = _trajGene->getVelPoly(_polyCoeff, traj_msg.num_segment - 1, _polyTime(traj_msg.num_segment - 1));  // 最后段的速度
  traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));  // 初始yaw角度
  traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));      // 最终yaw角度

  // 设置多项式的项数
  poly_number = traj_msg.num_order + 1;

  // 遍历每个轨迹段并填充轨迹系数
  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    for (unsigned int j = 0; j < poly_number; j++) {
      traj_msg.coef_x.push_back(polyCoeff(i, j) * pow(time(i), j));                // x方向的多项式系数
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) * pow(time(i), j));  // y方向的多项式系数
      traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j) * pow(time(i), j)); // z方向的多项式系数
    }
    traj_msg.time.push_back(time(i));                  // 设置该段的持续时间
    traj_msg.order.push_back(traj_msg.num_order);      // 设置多项式的阶数
  }
  traj_msg.mag_coeff = 1;  // 设置多项式的系数

  // 增加轨迹ID计数器，确保下一个轨迹消息的ID不同
  count++;

  // 发布轨迹消息
  ROS_WARN("[traj..gen...node] traj_msg publish");
  _traj_pub.publish(traj_msg);
}


// VectorXd timeAllocation(MatrixXd Path) {
//   VectorXd time(Path.rows() - 1);

// // 加速段时间(同时考虑加速和减速段）
//   double t_scope = 2.0*_Vel/_Acc;
//   // 加速段距离(同时考虑加速和减速段）
//   double distance_acc = 1.0 /2.0 * _Acc * t_scope * t_scope * 2.0;
  
//   for (int k = 0; k< Path.rows()-1; ++k){
//       Vector3d delta = Path.row(k) - Path.row(k + 1);
//       double d = std::sqrt(delta.dot(delta));

//       if(d <= distance_acc){
//           time(k) = std::sqrt(d/_Acc);
//       }
//       else{
//           time(k) = t_scope + (d - distance_acc)/_Vel;
//       }
//   }

//   return time;
// }
VectorXd timeAllocation(const MatrixXd Path) {
    // 检查路径点数是否足够
    if (Path.rows() < 2) {
        throw std::runtime_error("Path has insufficient points for time allocation.");
    }

    // 分配时间向量，大小为路径段数
    VectorXd time(Path.rows() - 1);

    // 计算加速和减速段的总时间以及加速段的距离
    double t_scope = 2.0 * _Vel / _Acc;  // 加速和减速所需时间
    double distance_acc = 0.5 * _Acc * t_scope * t_scope;  // 加速和减速段总距离

    for (int k = 0; k < Path.rows() - 1; ++k) {
        // 计算当前路径段的位移向量和长度
        Vector3d delta = Path.row(k + 1) - Path.row(k);
        double d = delta.norm();

        if (d <= distance_acc) {
            // 如果路径段短于加速段距离，则仅使用加速/减速段
            time(k) = 2.0 * std::sqrt(d / _Acc);  // 加速和减速时间
        } else {
            // 较长路径段，包含完整的加速、匀速和减速过程
            time(k) = t_scope + (d - distance_acc) / _Vel;  // 总时间 = 加速时间 + 匀速时间
        }
    }

    return time;
}



void visTrajectory(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  _traj_vis.header.frame_id = "world";

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 0.0;
  _traj_vis.color.g = 0.5;
  _traj_vis.color.b = 1.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_vis_pub.publish(_traj_vis);
}

void visPath(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_vis_pub.publish(points);
}

Vector3d getPos(double t_cur) {
  double time = 0;
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t);
        return pos;
      }
    }
  }
  return pos;
}

Vector3d getVel(double t_cur) {
  double time = 0;
  Vector3d Vel = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _trajGene->getVelPoly(_polyCoeff, i, t);
        return Vel;
      }
    }
  }
  return Vel;
}

int main(int argc, char **argv) {

  // 初始化ROS节点
  ros::init(argc, argv, "traj_node");
  ros::NodeHandle nh("~");

  // 获取参数
  nh.param("planning/vel", _Vel, 1.0);                // 规划中的最大速度
  nh.param("planning/acc", _Acc, 1.0);                // 规划中的最大加速度
  nh.param("planning/dev_order", _dev_order, 3);      // 规划中的导数阶数
  nh.param("planning/min_order", _min_order, 3);      // 规划中的最小阶数
  nh.param("vis/vis_traj_width", _vis_traj_width, 0.15); // 轨迹可视化宽度
  nh.param("map/resolution", _resolution, 0.2);       // 栅格地图分辨率
  nh.param("map/x_size", _x_size, 50.0);              // 地图x方向大小
  nh.param("map/y_size", _y_size, 50.0);              // 地图y方向大小
  nh.param("map/z_size", _z_size, 5.0);               // 地图z方向大小
  nh.param("path/resolution", _path_resolution, 0.01); // 路径分辨率
  nh.param("replanning/thresh_replan", replan_thresh, -1.0);    // 重规划触发距离阈值
  nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0); // 不触发重规划的距离阈值

  // 计算每个方向上的多项式系数个数
  _poly_num1D = 2 * _dev_order;

  // 设置定时器，以100Hz的频率调用execCallback函数
  _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);

  // 订阅ROS主题，包括当前的里程计状态、局部点云以及目标坐标
  _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);             // 订阅里程计信息
  _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack); // 订阅局部点云
  _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);     // 订阅目标点

  // 发布多项式轨迹、轨迹可视化和路径可视化
  _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50); // 发布轨迹
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);    // 发布轨迹可视化
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);          // 发布路径可视化

  // 设置障碍物地图的边界和参数
  _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;    // 地图的下边界
  _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size; // 地图的上边界
  _inv_resolution = 1.0 / _resolution;                  // 地图分辨率的倒数
  _max_x_id = (int)(_x_size * _inv_resolution);         // x方向上的最大栅格索引
  _max_y_id = (int)(_y_size * _inv_resolution);         // y方向上的最大栅格索引
  _max_z_id = (int)(_z_size * _inv_resolution);         // z方向上的最大栅格索引

  // 初始化A*路径搜索
  _astar_path_finder = new AstarPathFinder(); 
  _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

  // 循环运行ROS的回调函数
  ros::Rate rate(100);           // 设置循环频率为100Hz
  bool status = ros::ok();       // 检查ROS节点状态
  while (status) {
    ros::spinOnce();             // 处理所有已发布的回调
    status = ros::ok();          // 检查ROS节点是否仍在运行
    rate.sleep();                // 休眠以保持循环频率
  }

  return 0;                      // 程序正常结束
}
