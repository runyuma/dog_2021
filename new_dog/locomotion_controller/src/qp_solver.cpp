#include "qp_solver.h"
#define ABS(x) (x>0 ? x : -x)
#define SIGN(x) (x>0 ? 1 : -1)
qp_solver::qp_solver()
{
  D4<<120,  0,  0,  0,  0,  0,
        0, 40,  0,  0,  0,  0,
        0,  0, 20,  0,  0,  0,
        0,  0,  0, 80,  0,  0,
        0,  0,  0,  0,250,  0,
        0,  0,  0,  0,  0, 50;
  D2<< 60,  0,  0,  0,  0,  0,
        0, 30,  0,  0,  0,  0,
        0,  0,  2,  0,  0,  0,
        0,  0,  0, 80,  0,  0,
        0,  0,  0,  0,120,  0,
        0,  0,  0,  0,  0, 30;
    for (int i = 0; i < 4; i++)
    {
      R4(i*3,i*3) = 0.01;
      R4(i*3+1,i*3+1) = 0.01;
      R4(i*3+2,i*3+2) = 0.;
    }
    


}

int qp_solver::getlegnum()
{
  int sum = 0;
  for(int i=0;i<4;i++)
  {
    if(schedule_leg[i] == 1)
    {
      sum += 1;
    }
  }
  return sum;
}

void qp_solver::updateHessian(){
  int leg_num = getlegnum();
  Eigen::Matrix<float,12,12> _P = A.transpose()*D*A;
  if (leg_num == 4)
  {
    _P =_P+R4;
  }
  
//  std::cout<<_P.sparseView()<<std::endl;
  P = _P.sparseView();
}
void qp_solver::updateGradient(){
  Eigen::MatrixXf _Q = - A.transpose()*D*ForceTorque;
  Q = _Q.cast<double>();
}
void qp_solver::updateConstraints()
{
  int leg_num = getlegnum();
  Eigen::Matrix<float,20,12> _C = Eigen::Matrix<float,20,12>::Zero();
  Eigen::Matrix<float,12,12> RR  = Eigen::Matrix<float,12,12>::Zero();
  for(int i=0;i<4;i++)
    {
      RR.block(3*i,3*i,3,3) = R_mat;
    }
  for(int i=0;i<4;i++)
    {
      _C(i,i*3+2) = 1;
      _C(4+i*4,i*3) =1;
      _C(4+i*4+1,i*3) = 1;
      _C(4+i*4+2,i*3+1) = 1;
      _C(4+i*4+3,i*3+1)=1;
      _C(4+i*4,i*3+2) = -miu;
      _C(4+i*4+1,i*3+2) = miu;
      _C(4+i*4+2,i*3+2) = -miu;
      _C(4+i*4+3,i*3+2) = miu;
      if(schedule_leg[i] == 1)
      {

        if(leg_num ==4)
        {
          L(i) = 0.4*ForceTorque(2)/leg_num;
          U(i) = 1.5*ForceTorque(2)/leg_num;
        }
        else{
        L(i) = 0.6*ForceTorque(2)/leg_num;
        U(i) = 1.4*ForceTorque(2)/leg_num;
        }


      }
      else {
        L(i) = 0;
        U(i) = 0;
      }
      L(4+i*4)= -1000;
      U(4+i*4)= 0;
      L(4+i*4+1)= 0;
      U(4+i*4+1)= 1000;
      L(4+i*4+2)=-1000;
      U(4+i*4+2)=0;
      L(4+i*4+3)=0;
      U(4+i*4+3)=1000;
    }
  // _C = _C*RR;
  C = _C.sparseView();
}
int qp_solver::solveQP(Eigen::Matrix<float,3,4> & _foot_point, int *_schedule_leg, Eigen::VectorXf &_ForceTorque,Eigen::Matrix<float,3,3> &_R_mat,Eigen::Matrix<float,3,3> &_posture_mat)
// posture_mat z rotate not included
{
  OsqpEigen::Solver solver;
  // solver.settings()->setWarmStart(true);
  R_mat = _R_mat;
  solver.settings()->setVerbosity(false);
 // set the initial data of the QP solver
 int leg_num = getlegnum();
 ForceTorque = _ForceTorque;
 for(int i=0;i<4;i++)
 {
   schedule_leg[i] = _schedule_leg[i];
 }
 if(leg_num == 4)
 {
  D = D4;
 }
 else if(leg_num == 2 or leg_num == 3)
 {
   D = D2;
 }
 Eigen::MatrixXf Aupper(3,12),Alower(3,12);
 Aupper<<1,0,0,1,0,0,1,0,0,1,0,0,
         0,1,0,0,1,0,0,1,0,0,1,0,
         0,0,1,0,0,1,0,0,1,0,0,1;
  // std::cout<<"before"<<_foot_point<<std::endl;
  // _foot_point = _posture_mat * _foot_point;
  // std::cout<<"afteer"<<_foot_point<<std::endl;
 for(int i=0;i<4;i++)
 {
   Eigen::Matrix3f cross_product ;
  //  float footx;//TODO: mogai
  //  if(ABS( _foot_point(0,i)<=0.11))
  //  {
  //     footx = SIGN(_foot_point(0,i)) * 0.11;
  //  }
   cross_product(0,0 ) = 0;
   cross_product(1,0 ) = _foot_point(2,i);
   cross_product(2,0 ) = -_foot_point(1,i);
   cross_product(0,1 ) = -_foot_point(2,i);
   cross_product(1,1 ) = 0;
   cross_product(2,1) = _foot_point(0,i);
   cross_product(0,2 ) = _foot_point(1,i);
   cross_product(1,2 ) = -_foot_point(0,i);
   cross_product(2,2 ) = 0;
   Alower.block(0,3*i,3,3) = cross_product;
  //  Alower.block(0,3*i,3,3) = cross_product*_posture_mat;
 }
 A.block(0,0,3,12) = Aupper;
 A.block(3,0,3,12) =  Alower;

 updateHessian();
 updateGradient();
 updateConstraints();
  solver.data()->setNumberOfVariables(12);
  solver.data()->setNumberOfConstraints(20);
 if(!solver.data()->setHessianMatrix(P)) return 1;
 if(!solver.data()->setGradient(Q)) return 1;
 if(!solver.data()->setLinearConstraintsMatrix(C)) return 1;
 if(!solver.data()->setLowerBound(L)) return 1;
 if(!solver.data()->setUpperBound(U)) return 1;

 if(!solver.initSolver()) return 1;
 if(!solver.solve()) return 1;

 Eigen::MatrixXd sol;
 sol = solver.getSolution();

 Eigen::MatrixXf ans;
 ans = A * sol.cast<float>();
//  std::cout<<"ans"<<ans<<std::endl;
//   std::cout<<"A"<<A<<std::endl;
 Error = ans - ForceTorque;
 foot_force<<sol(0),sol(3),sol(6),sol(9),
            sol(1),sol(4),sol(7),sol(10),
            sol(2),sol(5),sol(8),sol(11);
 solver.data()->clearHessianMatrix();

// std::cout<<foot_force<<std::endl;
// std::cout<<Error<<std::endl;
}

void qp_solver::test()
{
  int _schedule_leg[4] = {1,0,0,1};
  Eigen::Matrix<float,3,4> _foot_point;
  Eigen::VectorXf _ForceTorque = Eigen::VectorXf::Zero(6);
  _ForceTorque<<0, 0, 150, -4.0, -5.0, 0.0;
  _foot_point<<0.1,-0.1,0.1,-0.1,
              -0.25,-0.25,0.25,0.25,
              -0.3,-0.3,-0.3,-0.3;
  // solveQP(_foot_point,_schedule_leg,_ForceTorque,Eigen::Matrix3f::Identity());
}

