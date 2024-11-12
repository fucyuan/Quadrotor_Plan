#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"
#include <geometry_msgs/Point.h>
#include "trajectory_generator_waypoint.h"

// 定义 AstarPathFinder 类，用于 A* 路径规划
class AstarPathFinder
{	
	private:  // 私有成员变量

	protected:  // 受保护成员变量和方法
		uint8_t * data; // 用于存储栅格地图的数据
		GridNodePtr *** GridNodeMap; // 三维指针数组，表示栅格地图中的所有节点
		Eigen::Vector3i goalIdx; // 终点的栅格索引
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE; // 地图在 x、y、z 方向的大小（以栅格为单位）
		int GLXYZ_SIZE, GLYZ_SIZE; // 地图的总体积和 yz 平面的大小

		double resolution, inv_resolution; // 地图的分辨率和分辨率倒数
		double gl_xl, gl_yl, gl_zl; // 地图在 x、y、z 方向的最小坐标
		double gl_xu, gl_yu, gl_zu; // 地图在 x、y、z 方向的最大坐标

		GridNodePtr terminatePtr; // 终止节点指针
		std::multimap<double, GridNodePtr> openSet; // 开放集合，用于存储候选节点，按照代价排序

		// 计算启发式代价，用于 A* 搜索
		double getHeu(GridNodePtr node1, GridNodePtr node2);

		// 获取当前节点的邻居节点和对应的边权重
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

    	// 判断节点是否被占用，重载了两个不同的函数
    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;

		// 判断节点是否空闲，重载了两个不同的函数
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
		
		// 将栅格索引转换为实际坐标
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);

		// 将实际坐标转换为栅格索引
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

	public:
		// 默认构造函数和析构函数
		AstarPathFinder(){};
		~AstarPathFinder(){};

		// A* 图搜索函数
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

		// 重置指定节点
		void resetGrid(GridNodePtr ptr);

		// 重置已使用的栅格节点
		void resetUsedGrids();

		// 初始化栅格地图
		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);

		// 设置指定坐标的栅格节点为障碍物
		void setObs(const double coord_x, const double coord_y, const double coord_z);

		// 对坐标进行四舍五入处理，得到离散化的栅格坐标
		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);

		// 获取计算出的路径
		std::vector<Eigen::Vector3d> getPath();

		// 获取访问过的节点集合
		std::vector<Eigen::Vector3d> getVisitedNodes();

		// 简化路径，删除多余节点
		std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d> &path, const double path_resolution);

		// 获取多项式轨迹在指定时间的位置信息
		Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);

		// 检查路径安全性
		int safeCheck(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);

		// 计算点到线的距离
		double disP2L(const Eigen::Vector3d& first, const Eigen::Vector3d& last, const Eigen::Vector3d& point);

};

#endif
