#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e) + robot_->getCoriolis() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    
    //calcolo della matrice d'inerzia
    Eigen::Matrix<double,7,7> B(robot_->getJsim());
    
    //calcolo della matrice dei guadagni
    Eigen::Matrix<double,6,6> Kp, Kd;
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();                                                              //parte posizionale di Kp
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();                                                              //parte rotazionale di Kp
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();                                                              //parte posizionale di Kd
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();                                                             //parte rotazionale di Kd
    
    // read current frame and twist    
    KDL::Frame curr_frame = robot_->getEEFrame();
    KDL::Twist curr_twist = robot_->getEEVelocity();
      
    
    
    // calculate errors
    Vector6d e, de;
    computeErrors(_desPos, curr_frame, _desVel, curr_twist, e , de);
    
    
    Eigen::VectorXd ddxd;
    ddxd.resize(6);
    ddxd << _desAcc.vel.x(), _desAcc.vel.y(), _desAcc.vel.z(),_desAcc.rot.x(), _desAcc.rot.y(), _desAcc.rot.z();
    
      
     
    
    return robot_->getJsim()*(pseudoinverse(robot_->getEEJacobian().data)*(ddxd + Kd*de + Kp*e - robot_->getEEJacDotqDot())) + robot_->getCoriolis() ;
    
}

