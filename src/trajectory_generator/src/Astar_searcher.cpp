#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2) + dz;

        if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
            neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
            neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
          continue;
        }

        if(isOccupied(neighborIdx(0),neighborIdx(1),neighborIdx(2))){
            continue;
        }

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  double h;
  Eigen::Vector3i start_index = node1->index;
  Eigen::Vector3i end_index = node2->index;


  double distance[3];
  distance[0] = abs((double)(start_index(0) - end_index(0)));
  distance[1] = abs((double)(start_index(1) - end_index(1)));
  distance[2] = abs((double)(start_index(2) - end_index(2)));
  std::sort(distance,distance+3);
  h = distance[0] + distance[1] + distance[2] +(std::sqrt(3.0)-3) * distance[0] + 
  (std::sqrt(2.0)-2)*distance[1];
  return h;
}


// 前端轨迹生成
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // 记录起点和终点对应的栅格坐标
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);

  // 目标索引
  goalIdx = end_idx;

  // 起点和终点的位置(应该可以省略)
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  // GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  // GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  // 初始化起点和终点节点(因为前面已经初始化过，所以不需要再New)
  GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
  GridNodePtr endPtr   = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];

  // 待弹出点集
  openSet.clear();

  // 定义要弹出的节点
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  //计算启发函数
  startPtr->gScore = 0;
  startPtr->fScore = getHeu(startPtr, endPtr);


  // 将起点加入开集
  // 自由点为0 闭集为-1 开集为1
  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  while (!openSet.empty()) {

    // 弹出最大f的节点
    currentPtr = openSet.begin()->second;
    currentPtr->id = -1; // 标记为闭集

    // 从开集中移除
    openSet.erase(openSet.begin());
    Eigen::Vector3i current_idx = currentPtr->index;


    // 获取拓展集合
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);     


    // 遍历拓展集合        
    for(int i = 0; i < (int)neighborPtrSets.size(); i++){
        

        neighborPtr = neighborPtrSets[i];
        double gh = currentPtr->gScore + edgeCostSets[i];
        double fh = gh + getHeu(neighborPtr,endPtr);

        // 如果为自由节点
        if(neighborPtr -> id == 0){ 
            
            // 计算相应的g和f，并加入opensets
            neighborPtr->gScore = gh;
            neighborPtr->fScore = fh;
            neighborPtr->cameFrom = currentPtr;

            neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));// 此处注意一定要先计算和赋值完f再加入

            // 判断是否为目标节点 改到此处为了提高代码效率 不用将所有节点加入后等弹出时发现目标再退出
            if(neighborPtr->index == goalIdx){
                ros::Time time_2 = ros::Time::now();
                terminatePtr = neighborPtr;
                ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );    
                return;
            }
            else{
                // 标记为open list
                neighborPtr->id = 1;
                continue;

            }
        }
        else if(neighborPtr ->id == 1){ 
            // 如果已经在openlist里面
            if(neighborPtr->gScore > gh)
            {
                // 更新对应的f值

                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr;
                openSet.erase(neighborPtr->nodeMapIt);
                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));


            }
        }
        else{
            // 如果是closelist里面的则不做处理
            continue;
        }
    }      

  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  GridNodePtr tmp_ptr = terminatePtr;
  while(tmp_ptr->cameFrom != NULL )
  {
      gridPath.push_back(tmp_ptr);
      tmp_ptr = tmp_ptr->cameFrom;
  }

  for (auto ptr: gridPath)
      path.push_back(ptr->coord);

      
  reverse(path.begin(),path.end());

  return path;
}


// 计算一点到一条直线（已知两点）的距离
//first和last分别为线的两端，third是第三点
double AstarPathFinder::disP2L(const Vector3d first, const Vector3d last, const Vector3d third) 
																//CMyPoint是点的类型，可以换成CPoint
{
  Vector3d nu = (third - first).cross(third - last);
  Vector3d de = first - last;
	double disSuqare = nu.norm()/de.norm();
	return disSuqare;
}


vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath;

  if (path.size() <= 2)
  {                            // 如果轨迹中的点数小于2，则直接返回原来的轨迹
    subPath = path;
  }else
  {
    const Vector3d first = path[0];                 //定义首点
	  const Vector3d last = path[path.size() - 1];    //定义尾点

    int flag = 0;                             //标记距离最大的点的下标
	  double disSquare = 0;

    for (int i = 1; i< path.size() - 1; i++) {
      double temp = disP2L(first, last, path[i]);
      if (temp > disSquare) {                    //记录最大距离及编号
        disSquare = temp;
        flag = i;
      }
    }

    if (disSquare < path_resolution) {                        //判断值与阈值的关系,阈值自己设定
      subPath.push_back(first);                               //如果小于阈值，则保留首尾点
      subPath.push_back(last);              
                                                              //用于存储留下来的点,是最后的成果
    }
    else {                                                    //否则分成两段
		vector<Vector3d> recResults1 = pathSimplify(vector<Vector3d>(path.begin(),path.begin()+flag), path_resolution);
    vector<Vector3d> recResults2 = pathSimplify(vector<Vector3d>(path.begin()+flag,path.end()), path_resolution);
    subPath.insert(subPath.end(),recResults1.begin(),recResults1.end()-1);
    subPath.insert(subPath.end(),recResults2.begin(),recResults2.end());
	  }
  }
  // return subPath;
  return subPath;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

// int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
//   int unsafe_segment = -1; //-1 -> the whole trajectory is safe
//   Vector3d pos;
//   for (int i = 0; i < time.size(); i++) {
//     for (double t = 0.0; t < time(i); t += 0.01) {
//       pos = getPosPoly(polyCoeff, i, t);
//       if(isOccupied(coord2gridIndex(pos))){
//         return i;
//       }

//     }
//   }
//   return unsafe_segment;
// }
int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
    int unsafe_segment = -1; //-1 -> the whole trajectory is safe
    Eigen::Vector3d pos;
    
    // 遍历每个轨迹段
    for (int i = 0; i < time.size(); i++) {
        bool segment_safe = true;

        // 在该段内进行更小步长的采样
        for (double t = 0.0; t < time(i); t += 0.005) {  // 更小的采样步长
            pos = getPosPoly(polyCoeff, i, t);
            Eigen::Vector3i grid_idx = coord2gridIndex(pos);

            // 检查当前位置以及周围的网格
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        Eigen::Vector3i neighbor_idx = grid_idx + Eigen::Vector3i(dx, dy, dz);
                        if (isOccupied(neighbor_idx)) {
                            // 发现了障碍物，则该段不安全
                            segment_safe = false;
                            return i;
                        }
                    }
                    if (!segment_safe) break;
                }
                if (!segment_safe) break;
            }

            if (!segment_safe) break;  // 如果段不安全，跳到下一个段
        }
    }
    
    return unsafe_segment;
}