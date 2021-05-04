#ifndef QP_SOLVER_H
#define QP_SOLVER_H
#include<Eigen/Dense>
#include "osqp.h"
#include "OsqpEigen/OsqpEigen.h"
class qp_solver
{
private:
  float miu = 0.25;
  int getlegnum();
  Eigen::Matrix<float,6,6> D2 = Eigen::Matrix<float,6,6>::Identity();
  Eigen::Matrix<float,6,6> D4 = Eigen::Matrix<float,6,6>::Identity();
  Eigen::Matrix<float,12,12>R4 = Eigen::Matrix<float,12,12>::Identity();
  Eigen::Matrix<float,12,12>R2 = Eigen::Matrix<float,12,12>::Identity();
public:
  Eigen::Matrix3f R_mat;
  Eigen::SparseMatrix<float> P ;
  Eigen::Matrix<float,6,12> A = Eigen::Matrix<float,6,12>::Zero();;
  Eigen::Matrix<float,6,6> D = Eigen::Matrix<float,6,6>::Identity();
  Eigen::VectorXd Q = Eigen::VectorXd::Zero(12);

  Eigen::VectorXd L = Eigen::VectorXd::Zero(20);
  Eigen::VectorXd U = Eigen::VectorXd::Zero(20);
//  q,l,u must be double

  Eigen::SparseMatrix<float> C;
//  constrain matrix

  int schedule_leg[4] = {1,1,1,1};
  Eigen::Matrix<float,3,4> foot_point;
  Eigen::VectorXf ForceTorque = Eigen::VectorXf::Zero(6);

  Eigen::Matrix<float,3,4> foot_force= Eigen::Matrix<float,3,4>::Zero();
  Eigen::VectorXf Error = Eigen::VectorXf::Zero(6);

  qp_solver();
  void updateHessian();
  void updateGradient();
  void updateConstraints();
  int solveQP(Eigen::Matrix<float,3,4> & _foot_point, int *_schedule_leg, Eigen::VectorXf &ForceTorque,Eigen::Matrix<float,3,3> &R_mat,Eigen::Matrix<float,3,3> &_posture_mat);
  void test();
};
#endif // QP_SOLVER_H
