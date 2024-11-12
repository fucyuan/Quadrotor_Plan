#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

// 定义无穷大值，用于初始化路径代价，1 << 20 表示2的20次方
#define inf 1>>20

// 声明GridNode结构体
struct GridNode;
typedef GridNode* GridNodePtr; // 定义指向GridNode的指针类型

// GridNode 结构体，用于表示栅格节点
struct GridNode
{     
    int id;        // 节点状态标识：1 --> open set (开放集), -1 --> closed set (闭合集)
    Eigen::Vector3d coord; // 节点在地图中的实际坐标
    Eigen::Vector3i dir;   // 扩展方向（从前一个节点到达当前节点的方向）
    Eigen::Vector3i index; // 节点在栅格地图中的索引位置
	
    double gScore, fScore; // gScore为从起点到当前节点的实际代价, fScore为估计总代价(gScore + hScore)
    double heuristic;      // 启发式函数的返回值，用于计算 fScore
    
    GridNodePtr cameFrom;  // 指向前驱节点的指针，用于回溯路径
    std::multimap<double, GridNodePtr>::iterator nodeMapIt; // 节点在 multimap 中的迭代器，用于优先队列中的操作

    // 构造函数，使用给定的索引和坐标初始化节点
    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;                   // 初始化节点状态为未处理状态
		index = _index;           // 设置节点的索引位置
		coord = _coord;           // 设置节点的实际坐标
		dir   = Eigen::Vector3i::Zero(); // 初始化方向为零向量

		gScore = inf;             // 初始化 gScore 为无穷大
		fScore = inf;             // 初始化 fScore 为无穷大
		cameFrom = NULL;          // 初始化前驱节点为空指针
    }

    GridNode(){
      double heuristic = -1; // 启发式函数的返回值，用于计算 fScore
    };                // 默认构造函数
    ~GridNode(){};               // 默认析构函数
};

#endif
