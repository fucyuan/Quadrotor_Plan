#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>



class TrajectoryGeneratorWaypoint {
private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;

        uint8_t * data;
        int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
        int GLXYZ_SIZE, GLYZ_SIZE;
        double resolution, inv_resolution;
        double gl_xl, gl_yl, gl_zl;
        double gl_xu, gl_yu, gl_zu;
        int Factorial(int x);
        void GetHession(const int n_seg, const int d_order,const Eigen::VectorXd &Time,Eigen::SparseMatrix<double> & hession);
        void GetLinearConstraintsMatrix(const int n_seg, const int d_order,const Eigen::VectorXd &Time,Eigen::SparseMatrix<double> & linearMatrix);
        void InsertCoff(const int row, const int col, Eigen::SparseMatrix<double> & linearMatrix , const double t , const int d_order, bool one_line , bool reverse );
public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time,
            OsqpEigen::Solver & slover);

        double getObjective();
        Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getVelPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getAccPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        
};

#endif
