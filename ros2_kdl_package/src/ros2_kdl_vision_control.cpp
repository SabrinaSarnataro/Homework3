#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "aruco/aruco.h"
#include <tf2_msgs/msg/tf_message.hpp>
#include <Eigen/Dense>


#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>


using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;




class VisionController : public rclcpp::Node {
public:
    VisionController() : Node("ros2_kdl_vision_control") , node_handle_(std::shared_ptr<VisionController>(this)){

        // declare cmd_interface parameter (velocity,effort)
        declare_parameter("cmd_interface", "velocity"); // defaults to "velocity"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

        declare_parameter<int>("point_2a", 1); 
        get_parameter("point_2a", point_2a);
        RCLCPP_INFO(get_logger(),"The choice of point 2 a) is: '%d'", point_2a);

        declare_parameter<int>("choice_dyn", 1); 
        get_parameter("choice_dyn", choice_dyn);
        RCLCPP_INFO(get_logger(),"The choice of inverse dynamic control is: '%d'", choice_dyn);

        if (!(cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
        {
            RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
        }

        if (!(point_2a == 1 || point_2a == 2))
        {
            RCLCPP_INFO(get_logger(),"Selected point_2a is not valid!"); return;
        }

        if (!(choice_dyn == 1 || choice_dyn == 2))
        {
            RCLCPP_INFO(get_logger(),"Selected choice_dyn is not valid!"); return;
        }

        // retrieve robot_description param
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});
            
            
        // create KDLrobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
        // Create joint array
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
        q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
        robot_->setJntLimits(q_min,q_max);            
        joint_positions_.resize(nj); 
        joint_velocities_.resize(nj); 
        joint_efforts_.resize(nj); 
        joint_pos.resize(nj);
        joint_vel.resize(nj);
    
        // Subscriber to jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&VisionController::joint_state_subscriber, this, std::placeholders::_1));

        // Wait for the joint_state topic
        while(!joint_state_available_){
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Update KDLrobot object
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data), toStdVector(joint_efforts_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data), toStdVector(joint_efforts_.data));
        joint_pos=joint_positions_; 
        joint_vel=joint_velocities_;


        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();
        std::cout<<"init_cart_pose "<<init_cart_pose_<<'\n';
       

        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);
            

        // Initialize controller
        controller_ = std::make_shared<KDLController>(*robot_);


        // EE's trajectory initial position (just an offset)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));

        // EE's trajectory end position
        Eigen::Vector3d end_position;  


        //Subscriber to aruco marker pose
        aruco_mark_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&VisionController::compute_pose_subscriber, this, std::placeholders::_1));

        // Wait for the joint_state topic
        while(!calc_traj)
        {
            std::cout<<calc_traj<<'\n';
            RCLCPP_INFO(this->get_logger(), "No aruco pose received yet! ...");
            rclcpp::spin_some(node_handle_);
        }


        //final point of the linear trajectory
        if(cmd_interface_ == "velocity"){
            end_position<<aruco_world_frame.p.data[0] + offset_x ,aruco_world_frame.p.data[1] + offset_y ,aruco_world_frame.p.data[2] + offset_z; 
        }
        else if( cmd_interface_ == "effort"){
            end_position << init_position[0]+0.2, init_position[1], init_position[2];
        }
        
        //std::cout<<"end_pos dopo :"<<end_position(0)<<" "<<end_position(1)<<" "<<end_position(2)<<"\n\n\n\n";        
                
        // Plan trajectory 
        double traj_duration = 1.5, acc_duration = 0.5;
                
        planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);//linear trajectory 

            
        if(cmd_interface_ == "velocity"){
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&VisionController::cmd_publisher, this));
            
            // Send joint velocity commands
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }
        }
        else{
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                            std::bind(&VisionController::cmd_publisher, this)); 
                                        
            error_pub_ = this->create_publisher<FloatArray>("/o_error", 10);
            Timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                            std::bind(&VisionController::error_pub, this));
                                            
            // Send joint effort commands
            for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                desired_commands_[i] = joint_efforts_(i);
            }
        }

        // Create msg and publish
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

            
        RCLCPP_INFO(this->get_logger(), "Vision Controller Node Started");
         
    }

private:
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_mark_pose_sub_ ;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::Publisher<FloatArray>::SharedPtr error_pub_;
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::TimerBase::SharedPtr Timer_;
    rclcpp::Node::SharedPtr node_handle_;


    bool camera_state_available_=false;
    bool calc_traj=false;
    bool joint_state_available_=false;
    bool calc_err=false;

    KDL::Frame aruco_world_frame;
    KDL::Frame init_cart_pose_;
    KDL::Frame cartpos;
    KDL::Frame aruco_cam_frame;

    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_efforts_;
    KDL::JntArray joint_pos;
    KDL::JntArray joint_vel;
   
    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<KDLController> controller_;
    std::string cmd_interface_;
    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    trajectory_point p;
    KDLPlanner planner_;

    int iteration_=0;
    int point_2a;
    int choice_dyn;

    double t_=0;
    double dt;
    double offset_x=0;
    double offset_y=0;
    double offset_z=0;
    double offset_angle=0;

    char rot_axis;

    float o_e;
    float l_e;   


    void cmd_publisher(){
        iteration_ = iteration_ + 1;

        // define trajectory
        double total_time = 1.5, total_time2=20; // 
        int trajectory_len = 150; // 
        int loop_rate = trajectory_len / total_time;
        dt = 1.0 / loop_rate;
        t_+=dt;

        Eigen::Vector3d s= toEigen(aruco_cam_frame.p)/ toEigen(aruco_cam_frame.p).norm();
        Eigen::Vector3d sd(0, 0, 1);            
        Eigen::Vector3d axis;
        double angle;
        KDL::Rotation align;
        KDL::Frame des_frame;

        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray ddqd , velocity_app;
        ddqd.resize(nj);
        velocity_app.resize(nj);

        if (t_ < total_time2){


            // Retrieve the trajectory point
            if(t_<=total_time)
                p = planner_.compute_trajectory(t_);
            else{
                p=planner_.compute_trajectory(total_time);;        
            }
                 
            // Compute EE frame
            cartpos = robot_->getEEFrame(); 
                    
            
            // compute errors
            Eigen::Vector3d error;
            Eigen::Vector3d o_error;
            Eigen::Matrix3d R_offset ;
                

            if(cmd_interface_ == "velocity" && point_2a ==1){

                
                if(rot_axis=='x'){
                    R_offset = Eigen::AngleAxisd(offset_angle,
                    Eigen::Vector3d::UnitX()).toRotationMatrix(); /
                }
                else if(rot_axis=='y'){
                    R_offset = Eigen::AngleAxisd(offset_angle, Eigen::Vector3d::UnitY()).toRotationMatrix(); 
                }
                else if(rot_axis=='z'){
                    R_offset = Eigen::AngleAxisd(offset_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
                }
                else if(rot_axis=='n'){
                    R_offset = Eigen::Matrix3d::Identity();
                }

                error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                o_error= computeOrientationError(R_offset*toEigen(aruco_world_frame.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;
                    
                // Compute differential IK
                Vector6d cartvel; cartvel << p.vel + 10*error, 10*o_error;
                joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;  
                    
            }
            else if(cmd_interface_ == "effort"){
                    
                //CONTROLLORE EFFORT JOINT SPACE

                angle= std::acos(s.dot(sd));
                axis= s.cross(sd);
                align = KDL::Rotation::Rot(toKDL(axis),angle);

                des_frame.M=(robot_->getEEFrame().M)*align;

                double Kp_=10, Kd_=5;
                double Kpp_=20, Kdp_=5, Kpo_= 1, Kdo_=5;
                   
                KDL::Frame desPos;
                KDL::Twist desVel, desAcc;

                error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                o_error = computeOrientationError(toEigen(des_frame.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                o_e=o_error.norm();
                l_e=error.norm();
                calc_err=true;


                if(choice_dyn == 2){

                    desPos.p= toKDL(p.pos);
                    desPos.M = des_frame.M;
                   
                    desVel.vel = toKDL(p.vel);
                    desVel.rot = {o_error[0] , o_error[1] , o_error[2] };

                    desAcc.vel = toKDL(p.acc);
                    desAcc.rot = {0.0 , 0.0 , 0.0 };
                    
                    joint_efforts_.data= controller_->idCntr(desPos, desVel,desAcc, Kpp_,Kpo_,Kdp_,Kdo_); 

                }
                else{

                    velocity_app= joint_vel;
                    
                    Vector6d des_vel; 
                    des_vel << p.vel + 5*error, 1*o_error;
                    joint_vel.data = pseudoinverse(robot_->getEEJacobian().data)*des_vel;
                    joint_pos.data = joint_pos.data + joint_vel.data*dt;

                    ddqd.data=(joint_vel.data-velocity_app.data)/dt; 
                    joint_efforts_.data= controller_->idCntr(joint_pos, joint_vel, ddqd , Kp_ , Kd_);

                }
                    calc_traj=false;
                   
            }

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data), toStdVector(joint_efforts_.data));

            if(cmd_interface_ == "velocity"){
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else{
                // Send joint effort commands
                //std::cout<< "Data size "<< joint_efforts_.data.size() << std::endl;
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
               
        }
        else{
            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
               
            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }

        if(cmd_interface_ == "velocity" && point_2a == 2){

            Eigen::Matrix3d I3 =  Eigen::Matrix3d::Identity();
            Eigen::MatrixXd L  = Eigen::Matrix<double,3,6>::Zero();
            L.block(0,0,3,3) = (-1/toEigen(aruco_cam_frame.p).norm())*(I3 - s*s.transpose());
            L.block(0,3,3,3) = skew(s);                                                                                      
            Eigen::Matrix3d Rc = toEigen(robot_->getEEFrame().M);
            Eigen::MatrixXd R = Eigen::Matrix<double,6,6>::Zero();
            R.block(0,0,3,3) = Rc;
            R.block(3,3,3,3) = Rc;
            L=L*R;                                                                                                          

            Eigen::MatrixXd I7 =  Eigen::MatrixXd::Identity(7,7);
            Eigen::MatrixXd LJ = L*robot_->getEEJacobian().data;
            Eigen::MatrixXd LJpinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd N = (I7 - (LJpinv*LJ));
            double k = 2;

            joint_velocities_.data = k*LJpinv*sd+ N*(joint_pos.data - joint_positions_.data);    
            joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
            calc_traj=false;

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data), toStdVector(joint_efforts_.data));
       
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

        }
   
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
            joint_efforts_.data[i] = sensor_msg.effort[i];
        }
    }


    void compute_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {

        if(!calc_traj)
        {

            KDL::Rotation y_rotation = KDL::Rotation::RotY(M_PI);
            KDL::Rotation z_rotation = KDL::Rotation::RotZ(-M_PI/2);

            KDL::Vector aruco_trasl_cam_frame;
            Eigen::VectorXd aruco_quat_cam_frame;

            aruco_quat_cam_frame.resize(4);

            if(cmd_interface_ == "velocity" && point_2a == 1){
                std::cout << "WARNING! Do not enter excessively large offset values to avoid straining the joints and exceeding joint limits." << std::endl;
                std::cout << "If the robot behaves strangely, it is because unacceptable values were entered. Please try again. " << std::endl;
                std::cout <<  "The optimal choice is x=0 y=0.5 z=0)" << std::endl;
                std::cout << "Insert the position offset on the x-axis (double):" << std::endl;
                std::cin >> offset_x ;
                std::cout << "Insert the position offset on the y-axis (double):" << std::endl;
                std::cin >> offset_y ;
                std::cout << "Insert the position offset on the z-axis (double):" << std::endl;
                std::cin >> offset_z ;


                while(!(rot_axis== 'x' || rot_axis== 'y' || rot_axis== 'z' || rot_axis== 'n' ))              
                {
                    std::cout<<"Insert the axis around which you want to perform the rotation (x/y/z/n):" << std::endl<< "choice n if you don't want the rotation" << std::endl;
                    std::cin>>rot_axis;
                    if(!(rot_axis== 'x' || rot_axis== 'y' || rot_axis== 'z' || rot_axis== 'n' ))
                        RCLCPP_INFO(get_logger(),"Selected axes is not valid!");

                }
                if (rot_axis != 'n'){
                    std::cout << "Insert the orientation offset (double):" << std::endl;
                    std::cin >> offset_angle ;

                }
            }

            aruco_trasl_cam_frame[0] = pose_msg->pose.position.x ;
            aruco_trasl_cam_frame[1] = pose_msg->pose.position.y ;
            aruco_trasl_cam_frame[2] = pose_msg->pose.position.z ;
                
            aruco_quat_cam_frame[0] = pose_msg->pose.orientation.x;
            aruco_quat_cam_frame[1] = pose_msg->pose.orientation.y;
            aruco_quat_cam_frame[2] = pose_msg->pose.orientation.z;
            aruco_quat_cam_frame[3] = pose_msg->pose.orientation.w;
                
            aruco_cam_frame.M = KDL::Rotation::Quaternion(aruco_quat_cam_frame[0], aruco_quat_cam_frame[1], aruco_quat_cam_frame[2], aruco_quat_cam_frame[3]);
                
            aruco_cam_frame.M=aruco_cam_frame.M* y_rotation*z_rotation;
                
            aruco_cam_frame.p = aruco_trasl_cam_frame;
                
            if(cmd_interface_ == "velocity"){
                aruco_world_frame=  aruco_cam_frame;
            }
            else if(cmd_interface_ == "effort"){
                aruco_world_frame= robot_->getEEFrame()*aruco_cam_frame;
            }
     
            calc_traj=true;
            //std::cout<<"endpos"<<aruco_world_frame.p.data[0]<< " " <<aruco_world_frame.p.data[1]<< " " <<aruco_world_frame.p.data[2]<<"\n";
            
        }
        else{
            std::cout << "After compute aruCo's pose " << std::endl;

        }


    }

    void error_pub(){
       if(calc_err==true){
        std_msgs::msg::Float64MultiArray error_msg;
        error_msg.data.resize(2);
            
        error_msg.data[0]=l_e;
        error_msg.data[1]=o_e;
       	error_pub_->publish(error_msg);
       	calc_err=false;

       }
    }

};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionController>());
    rclcpp::shutdown();
    return 0;
}








