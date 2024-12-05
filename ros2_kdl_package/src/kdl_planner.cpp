#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}


void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;
  double s, sd, sdd;
  
  switch(choice_traj){
    
    case 1:
      cubic_polynomial(time, s, sd, sdd);
      //circular trajectory
      traj.pos[0]= trajInit_[0];
      traj.pos[1]= trajInit_[1] + trajRadius_ - trajRadius_*cos(2*M_PI*s);
      traj.pos[2]= trajInit_[2] - trajRadius_*sin(2*M_PI*s);
      traj.vel[0]= 0;
      traj.vel[1]= 2*M_PI*sd*trajRadius_*sin(2*M_PI*s);
      traj.vel[2]= - 2*M_PI*sd*trajRadius_*cos(2*M_PI*s);
      traj.acc[0]= 0;
      traj.acc[1]= 2*M_PI*sdd*trajRadius_*sin(2*M_PI*s) + std::pow(2*M_PI*sd,2)*trajRadius_*cos(2*M_PI*s);
      traj.acc[2]= - 2*M_PI*sdd*trajRadius_*cos(2*M_PI*s) + std::pow(2*M_PI*sd,2)*trajRadius_*sin(2*M_PI*s); 
    break;
    
    case 2:
      cubic_polynomial(time, s, sd, sdd);
      //linear trajectory
      traj.pos = trajInit_ + s*(trajEnd_ - trajInit_); //non c'è bisogno di dividere per la norma perchè s varia tra 0 e 1
      traj.vel = sd*(trajEnd_ - trajInit_);
      traj.acc = sdd*(trajEnd_ - trajInit_);
    break;
   
   
    case 3:
      trapezoidal_vel(time, accDuration_, s, sd, sdd);
     
      //circular trajectory
      traj.pos[0]= trajInit_[0];
      traj.pos[1]= trajInit_[1] + trajRadius_ - trajRadius_*cos(2*M_PI*s);
      traj.pos[2]= trajInit_[2] - trajRadius_*sin(2*M_PI*s);
      traj.vel[0]= 0;
      traj.vel[1]= 2*M_PI*sd*trajRadius_*sin(2*M_PI*s);
      traj.vel[2]= - 2*M_PI*sd*trajRadius_*cos(2*M_PI*s);
      traj.acc[0]= 0;
      traj.acc[1]= 2*M_PI*sdd*trajRadius_*sin(2*M_PI*s) + std::pow(2*M_PI*sd,2)*trajRadius_*cos(2*M_PI*s);
      traj.acc[2]= - 2*M_PI*sdd*trajRadius_*cos(2*M_PI*s) + std::pow(2*M_PI*sd,2)*trajRadius_*sin(2*M_PI*s); 

    break;
    
    
    case 4:
      trapezoidal_vel(time, accDuration_, s, sd, sdd);
      //linear trajectory
      traj.pos = trajInit_ + s*(trajEnd_ - trajInit_); 
      traj.vel = sd*(trajEnd_ - trajInit_);
      traj.acc = sdd*(trajEnd_ - trajInit_);
    break;
    
    
    
    default: 
      Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);
      if(time <= accDuration_)
      {
        traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
        traj.vel = ddot_traj_c*time;
        traj.acc = ddot_traj_c;
      }
      else if(time <= trajDuration_-accDuration_)
      {
        traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
        traj.vel = ddot_traj_c*accDuration_;
        traj.acc = Eigen::Vector3d::Zero();
      }
      else
      {
        traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
        traj.vel = ddot_traj_c*(trajDuration_-time);
        traj.acc = -ddot_traj_c;
      }
    break;
    
  }
  
  return traj;

}

void KDLPlanner::trapezoidal_vel(double t, double tc, double &s, double &sd, double &sdd){
  double scdd= 8/std::pow(trajDuration_,2); //NOTA: scdd deve essere maggiore di 4*|sf-si|/pow(trajDuration_,2) , sf=1, si=0
   
  if(t<=tc){
    s= 0.5*scdd*std::pow(t,2);
    sd= scdd*t;
    sdd= scdd;
  }
  else if(tc<t && t<= (trajDuration_-tc)){
    s=scdd*tc*(t-0.5*tc);
    sd=scdd*tc;
    sdd=0;
  }
  else if ((trajDuration_-tc)< t && t <= trajDuration_){
    s= 1-0.5*scdd*std::pow(trajDuration_-t,2);
    sd=scdd*(trajDuration_-t);
    sdd=-scdd;
  }
    //std::cout << s <<std::endl;

}
void KDLPlanner::cubic_polynomial(double t, double &s, double &sd, double &sdd){
  double a0,a1,a2,a3;
  
  a0=0;
  a1=0;
  a2=3/std::pow(trajDuration_,2);
  a3= -2/std::pow(trajDuration_,3);
  
  s=a3*std::pow(t,3)+a2*std::pow(t,2)+a1*t+a0;
  sd=3*a3*std::pow(t,2)+2*a2*t+a1;
  sdd=6*a3*t+2*a2;
  

}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius, double _accDuration){
  trajDuration_ = _trajDuration;
  trajInit_ = _trajInit;
  trajRadius_ = _trajRadius;
  accDuration_=_accDuration;

}




