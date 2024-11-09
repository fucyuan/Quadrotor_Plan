#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

// 计算多项式的阶乘
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

// 获取hession矩阵，也就是矩阵Q
void TrajectoryGeneratorWaypoint::GetHession(const int n_seg, 
                const int d_order,
                const Eigen::VectorXd &Time,
                Eigen::SparseMatrix<double> & hession){
    
    int p_order = 2 * d_order - 1;
    int p_num1d = p_order + 1;
    hession.resize(n_seg * p_num1d,n_seg * p_num1d);

    hession.setZero();

    for(int k = 0; k < n_seg; ++k){

        for(int i = d_order; i < p_num1d; ++i){
            for(int j = d_order; j < p_num1d ; ++j){
                double value = 1.0*Factorial(i)/Factorial(i-d_order)*Factorial(j)/Factorial(j-d_order)/(i+j-2*d_order+1)*pow(Time(k),i+j-2*d_order+1);
                hession.insert(k*p_num1d + i,k*p_num1d + j) = value;
            }
        }
        //hession.block( k*p_num1d,k*p_num1d,p_num1d,p_num1d) = Q_k; 稀疏矩阵不支持
    }
}

// 在线性约束矩阵的指定位置插入系数
void TrajectoryGeneratorWaypoint::InsertCoff(const int row, 
                const int col, 
                Eigen::SparseMatrix<double> & linearMatrix , 
                const double t , 
                const int d_order,
                bool one_line ,
                bool reverse){
    
    int p_num1d = 2*d_order;

    int flag = d_order ;
    if(one_line){
        flag = 1;
    }

    Eigen::MatrixXd coff(d_order,p_num1d);

    if(d_order == 4){
        coff << 1.0, 1.0*t,1.0*pow(t,2),1.0*pow(t,3),1.0*pow(t,4),1.0*pow(t,5),1.0*pow(t,6),1.0*pow(t,7),
                0.0, 1.0, 2.0*t, 3.0*pow(t,2), 4.0*pow(t,3), 5.0*pow(t,4), 6.0*pow(t,5), 7.0*pow(t,6),
                0.0, 0.0, 2.0, 6.0*t, 12.0*pow(t,2), 20.0*pow(t,3), 30.0*pow(t,4), 42.0*pow(t,5),
                0.0, 0.0, 0.0, 6.0, 24.0*t, 60.0*pow(t,2), 120.0*pow(t,3),210.0*pow(t,4);
    }
    else if(d_order == 3){
        coff << 1.0, 1.0*t,1.0*pow(t,2),1.0*pow(t,3),1.0*pow(t,4),1.0*pow(t,5),
                0.0, 1.0, 2.0*t, 3.0*pow(t,2), 4.0*pow(t,3), 5.0*pow(t,4),
                0.0, 0.0, 2.0, 6.0*t, 12.0*pow(t,2), 20.0*pow(t,3);
    }else{
        cout << "暂时只支持minisnap和minijerk";
    }

    if(reverse){
        coff = coff *(-1.0);
    }

    for (int i = 0; i < d_order && i < flag; ++i){
        for(int j = 0; j < p_num1d; ++j){
            linearMatrix.insert(row + i,col + j) = coff(i,j);
        }
    }

}

// 获取等式约束矩阵，也就是矩阵Aeq
void TrajectoryGeneratorWaypoint::GetLinearConstraintsMatrix(const int n_seg,
                const int d_order,
                const Eigen::VectorXd &Time,
                Eigen::SparseMatrix<double> & linearMatrix){

    int p_order = 2 * d_order - 1;
    int p_num1d = p_order + 1;

    linearMatrix.resize(2*d_order + (n_seg-1)*( d_order + 1),p_num1d * n_seg);

    // 起点和终点限制约束
    int row = 0;
    int col = 0;
    InsertCoff(row,col,linearMatrix,0,d_order,false,false);

    row += d_order;
    col = (n_seg - 1) * p_num1d;
    InsertCoff(row,col,linearMatrix,Time(n_seg-1),d_order,false,false);


    // 中间节点的位置约束
    row += d_order;
    for(int k = 0; k < n_seg -1; ++k){
        InsertCoff(row + k , k*p_num1d ,linearMatrix,Time(k),d_order,true,false);
    }
    // 连续性约束
    row += n_seg - 1;
    for(int k = 0; k < n_seg - 1; ++k){
        InsertCoff(row, k*p_num1d ,linearMatrix,Time(k),d_order,false,false);
        InsertCoff(row, (k + 1)*p_num1d ,linearMatrix,0,d_order,false,true);
        row += d_order;
    }

}


// Minisnap轨迹优化
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time,
    OsqpEigen::Solver & slover) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  
  MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);            // position(x,y,z), so we need (3 * p_num1d) coefficients

  VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

  // 设置变量个数
  slover.data()->setNumberOfVariables( m * p_num1d );

  //设置约束个数
  slover.data()->setNumberOfConstraints(d_order*2 + (m - 1)*(d_order+1) );

  // 设置H矩阵
  Eigen::SparseMatrix<double> hession;
  GetHession(m,d_order,Time,hession);
  if(!slover.data()->setHessianMatrix(hession)){
      cout << "设置hession矩阵失败";
      return Eigen::MatrixXd::Zero(1,1);
  }

  //设置线性约束矩阵
  Eigen::SparseMatrix<double> linearMatrix;
  GetLinearConstraintsMatrix(m,d_order,Time,linearMatrix);

  if(!slover.data()->setLinearConstraintsMatrix(linearMatrix)){
      cout << "设置Linear矩阵失败";
      return Eigen::MatrixXd::Zero(1,1);
  }

  Eigen::VectorXd gradient(p_num1d * m);
  gradient.setZero();

  // 设置梯度约束
  if(!slover.data()->setGradient(gradient)){
      cout << "梯度设置失败" <<endl;
  }

  // 设置边界，求解问题
  Eigen::VectorXd lowbound = VectorXd::Zero(d_order*2 + (m - 1)*(d_order+1));
  Eigen::VectorXd upbound = VectorXd::Zero(d_order*2 + (m - 1)*(d_order+1));

  slover.data()->setLowerBound(lowbound);
  slover.data()->setUpperBound(upbound);

  //初始化求解器
  if(!slover.isInitialized()){
      slover.initSolver();
  }

  for(int dim = 0; dim < 3; dim++){

    VectorXd wayPoints = Path.col(dim);
    

    // 起点位置
    lowbound(0) = wayPoints(0);
    upbound(0) = wayPoints(0);

    // 终点位置
    lowbound(d_order) = wayPoints(m);
    upbound(d_order) = wayPoints(m);

    // 固定中间节点位置
    for(int i = 0; i < m - 1; i++ ){
        lowbound(2*d_order + i) = wayPoints(i + 1);
        upbound(2*d_order + i) = wayPoints(i + 1);
    }
    
    // 更新边界
    slover.updateBounds(lowbound,upbound);

    // 求解
    slover.solve();

    Eigen::VectorXd poly_coef_1d = slover.getSolution();


    MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();


    for(int k = 0; k < m; k++){
        PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0,k*p_num1d, 1, p_num1d);
    }
  }

  // 每次调用之后需要清理变量
  slover.data()->clearHessianMatrix();
  slover.data()->clearLinearConstraintsMatrix();
  slover.clearSolverVariables();
  slover.clearSolver();

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
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

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}