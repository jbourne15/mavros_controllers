//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "geometric_controller/geometric_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  max_motor_speed_(150),
  fail_detec_(false),
  ctrl_enable_(true),
  landing_commanded_(false),
  ctrl_mode_(2){

  /// Target State is the reference state received from the trajectory
  /// goalState is the goal the controller is trying to reach
  goalPos_ << 0.0, 0.0, 1.5; //Initial Position
  targetPos_ = goalPos_;
  targetVel_ << 0.0, 0.0, 0.0;
  mavYaw_ = 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -3.0, -5.0, -40.0;
  Kvel_ << -2.0, -4.0, -5.0;
  D_ << 0.0, 0.0, 0.0;
  attctrl_tau_ = 0.2;
  norm_thrust_const_ = .1;//0.1; // 1 / max acceleration
  max_fb_acc_ = 7.0;
  use_gzstates_ = false;//true;//true;

  agentName = ros::this_node::getNamespace();  
  agentName.erase(0,1);
  newVelData=false;

  rcSub_ = nh_.subscribe(agentName+"/mavros/rc/in",1,&geometricCtrl::rc_command_callback,this,ros::TransportHints().tcpNoDelay());
  referenceSub_=nh_.subscribe(agentName+"/reference/setpoint",1, &geometricCtrl::targetCallback,this,ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe(agentName+"/reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
  mavstateSub_ = nh_.subscribe(agentName+"/mavros/state", 1, &geometricCtrl::mavstateCallback, this,ros::TransportHints().tcpNoDelay());
  // mavposeSub_ = nh_.subscribe(agentName+"/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe(agentName+"/local_position", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  
  mavtwistSub_ = nh_.subscribe(agentName+"/mavros/local_position/velocity", 1, &geometricCtrl::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
   // mavtwistSub_ = nh_.subscribe(agentName+"/local_velocity", 1, &geometricCtrl::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  
    gzmavposeSub_ = nh_.subscribe("/gazebo/model_states", 1, &geometricCtrl::gzmavposeCallback, this, ros::TransportHints().tcpNoDelay());
  ctrltriggerServ_ = nh_.advertiseService(agentName+"/tigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback, this); // Define timer for constant loop rate

  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(agentName+"/command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>(agentName+"/reference/pose", 1);

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(agentName+"/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(agentName+"/mavros/set_mode");
  
  frame_client    = nh_.serviceClient<mavros_msgs::SetMavFrame>(agentName+"/mavros/mav_frame");
  
  mav_frame.request.mav_frame = mav_frame.request.FRAME_BODY_NED;
  frame_client.call(mav_frame);

  nh_.param<string>("/geometric_controller/mavname", mav_name_, "iris");
  nh_.param<int>("/geometric_controller/ctrl_mode", ctrl_mode_, MODE_BODYRATE);
  nh_.param<bool>("/geometric_controller/enable_sim", sim_enable_, true);

  // ros::Duration(15.0).sleep(); // sleep for half a second
}
geometricCtrl::~geometricCtrl() {
  //Destructor
}

void geometricCtrl::rc_command_callback(const mavros_msgs::RCIn::ConstPtr &new_message)
{
  RCin = *new_message;
  mode = RCin.channels[4];
}


void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped& msg) {

  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_; // this needs to have CA added to it!!!!!!

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
  targetVel_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;

  if (targetVel_.norm()>=0.25){
    mavYaw_ = std::atan2(targetVel_(1),targetVel_(0));
  }
  else{
    mavYaw_ = tf::getYaw(tf::Quaternion(mavAtt_(1), mavAtt_(2), mavAtt_(3), mavAtt_(0)));
  }

  mavYaw_ = 0;
    
  if(reference_request_dt_ > 0) targetAcc_ = (targetVel_ - targetVel_prev_ ) / reference_request_dt_; // I can get this from trajectory!!!
  else targetAcc_ = Eigen::Vector3d::Zero();

}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget& msg) {

  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_; 

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << msg.position.x, msg.position.y, msg.position.z;
  targetVel_ << msg.velocity.x, msg.velocity.y, msg.velocity.z;

  if(msg.type_mask == 1) {

    targetAcc_ << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;
    targetJerk_ << msg.jerk.x, msg.jerk.y, msg.jerk.z;
    targetSnap_ << 0.0, 0.0, 0.0;

  } else if (msg.type_mask == 2) {

    targetAcc_ << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;
    targetJerk_ << 0.0, 0.0, 0.0;
    targetSnap_ << 0.0, 0.0, 0.0;

  } else if (msg.type_mask == 4) {

    targetAcc_ << 0.0, 0.0, 0.0;
    targetJerk_ << 0.0, 0.0, 0.0;
    targetSnap_ << 0.0, 0.0, 0.0;

  } else {

    targetAcc_ << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;
    targetJerk_ << msg.jerk.x, msg.jerk.y, msg.jerk.z;
    targetSnap_ << msg.snap.x, msg.snap.y, msg.snap.z;

  }
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped& msg){
  if(!use_gzstates_){
    mavPos_(0) = msg.pose.position.x;
    mavPos_(1) = msg.pose.position.y;
    mavPos_(2) = msg.pose.position.z;
    mavAtt_(0) = msg.pose.orientation.w;
    mavAtt_(1) = msg.pose.orientation.x;
    mavAtt_(2) = msg.pose.orientation.y;
    mavAtt_(3) = msg.pose.orientation.z;
  }
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped& msg){
  if(!use_gzstates_) {
    mavVel_(0) = msg.twist.linear.x;
    mavVel_(1) = msg.twist.linear.y;
    mavVel_(2) = msg.twist.linear.z;
    mavRate_(0) = msg.twist.angular.x;
    mavRate_(1) = msg.twist.angular.y;
    mavRate_(2) = msg.twist.angular.z;
    newVelData=true;
  }
}

void geometricCtrl::gzmavposeCallback(const gazebo_msgs::ModelStates& msg){
  //TODO: gazebo_msgs should not be compiled on the vehicle
  if(use_gzstates_){
    for(int i = 0; i < msg.pose.size(); i++){
      if(msg.name[i] == mav_name_){
        mavPos_(0) = msg.pose[i].position.x;
        mavPos_(1) = msg.pose[i].position.y;
        mavPos_(2) = msg.pose[i].position.z;
        mavAtt_(0) = msg.pose[i].orientation.w;
        mavAtt_(1) = msg.pose[i].orientation.x;
        mavAtt_(2) = msg.pose[i].orientation.y;
        mavAtt_(3) = msg.pose[i].orientation.z;
        mavVel_(0) = msg.twist[i].linear.x;
        mavVel_(1) = msg.twist[i].linear.y;
        mavVel_(2) = msg.twist[i].linear.z;
        mavRate_(0) = msg.twist[i].angular.x;
        mavRate_(1) = msg.twist[i].angular.y;
        mavRate_(2) = msg.twist[i].angular.z;
      }
    }
  }
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent& event){
  if(sim_enable_ && mode<1100){
    // Enable OFFBoard mode and arm automatically
    // This is only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
        if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }

  //TODO: Enable Failsaif sutdown
  if(ctrl_mode_ == MODE_ROTORTHRUST){
    //TODO: Compute Thrust commands
  } else if(ctrl_mode_ == MODE_BODYRATE){
      computeBodyRateCmd(false);
      pubReferencePose();
      pubRateCommands();
  } else if(ctrl_mode_ == MODE_BODYTORQUE){
    //TODO: implement actuator commands for mavros
  }
  ros::spinOnce();
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void geometricCtrl::statusloopCallback(const ros::TimerEvent& event){

}

void geometricCtrl::pubReferencePose(){
  referencePoseMsg_.header.stamp = ros::Time::now();
  referencePoseMsg_.header.frame_id = "world";
  referencePoseMsg_.pose.position.x = targetPos_(0);
  referencePoseMsg_.pose.position.y = targetPos_(1);
  referencePoseMsg_.pose.position.z = targetPos_(2);
  referencePoseMsg_.pose.orientation.w = q_des(0);
  referencePoseMsg_.pose.orientation.x = q_des(1);
  referencePoseMsg_.pose.orientation.y = q_des(2);
  referencePoseMsg_.pose.orientation.z = q_des(3);
  referencePosePub_.publish(referencePoseMsg_);
}

void geometricCtrl::pubRateCommands(){
  angularVelMsg_.header.stamp = ros::Time::now();
  angularVelMsg_.header.frame_id= "world";
  angularVelMsg_.body_rate.x = cmdBodyRate_(0);
  angularVelMsg_.body_rate.y = cmdBodyRate_(1) * -1.0;
  angularVelMsg_.body_rate.z = cmdBodyRate_(2) * -1.0;
  angularVelMsg_.type_mask = 128; //Ignore orientation messages
  angularVelMsg_.thrust = cmdBodyRate_(3);
  angularVelPub_.publish(angularVelMsg_);
}

void geometricCtrl::computeBodyRateCmd(bool ctrl_mode){
  Eigen::Vector3d errorPos_, errorVel_;
  Eigen::Matrix3d R_ref;

  errorPos_ = mavPos_ - targetPos_;
  errorVel_ = mavVel_ - targetVel_;
  a_ref = targetAcc_;

  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  R_ref = quat2RotMatrix(q_ref);
  a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_; //feedforward term for trajectory error
  if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;
  a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * targetVel_; //Rotor drag
  a_des = a_fb + a_ref - a_rd - g_;
  q_des = acc2quaternion(a_des, mavYaw_);

  // std::cout<<"pos:"<<std::endl;
  // std::cout<<errorPos_[2]<<std::endl;
  // std::cout<<"vel:"<<std::endl;
  // std::cout<<errorVel_<<std::endl;
  // std::cout<<"acc:"<<std::endl;
  // std::cout<<a_ref<<std::endl;  
  
  cmdBodyRate_ = attcontroller(q_des, a_des, mavAtt_); //Calculate BodyRate
}

Eigen::Vector4d geometricCtrl::quatMultiplication(Eigen::Vector4d &q, Eigen::Vector4d &p) {
   Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d geometricCtrl::quat2RotMatrix(Eigen::Vector4d q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d geometricCtrl::rot2Quaternion(Eigen::Matrix3d R){
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(Eigen::Vector3d vector_acc, double yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, yc;
  Eigen::Matrix3d rotmat;
  yc = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*Eigen::Vector3d::UnitY();
  zb_des = vector_acc / vector_acc.norm();
  xb_des = yc.cross(zb_des) / ( yc.cross(zb_des) ).norm();
  yb_des = zb_des.cross(xb_des) / (zb_des.cross(xb_des)).norm();
  rotmat << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);  
  return quat;
}

Eigen::Vector4d geometricCtrl::attcontroller(Eigen::Vector4d &ref_att, Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att){
  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb;
  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * curr_att;
  qe = quatMultiplication(q_inv, ref_att);
  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);
  // ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb))); //Calculate thrust
  ratecmd(3) = std::max(0.0, norm_thrust_const_ * ref_acc.dot(zb)); //Calculate thrust
  return ratecmd;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
}
