//
// Created by jalim on 05.05.18.
//

#include "trajectory_publisher/trajectoryPublisher.h"

using namespace std;
using namespace Eigen;
trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  motionPrimitives_(0.1) {

  nh_.setParam("/uav1/quad_vis/marker_scale", 1.5);
  
  trajectoryPub_ = nh_.advertise<nav_msgs::Path>("reference/trajectory", 1);
  referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
  accelReference_pub = nh_.advertise<geometry_msgs::AccelStamped>("reference/accel", 1);
  markers_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory", 1);

  agentName = ros::this_node::getNamespace();  
  agentName.erase(0,1);

  local_sub   = nh_.subscribe(agentName+"/gps_pose", 1, &trajectoryPublisher::localCallback, this, ros::TransportHints().tcpNoDelay()); // from geodetic
  state_sub   = nh_.subscribe(agentName+"/mavros/state", 1, &trajectoryPublisher::stateCallback, this, ros::TransportHints().tcpNoDelay());
  
  trajloop_timer_ = nh_.createTimer(ros::Duration(1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_.param<double>("trajectory_publisher/initpos_x", init_pos_x_, 0.0);
  nh_.param<double>("trajectory_publisher/initpos_y", init_pos_y_, 0.0);
  nh_.param<double>("trajectory_publisher/initpos_z", init_pos_z_, 1.0);
  nh_.param<double>("trajectory_publisher/updaterate", controlUpdate_dt_, 0.01);
  nh_.param<int>("trajectory_publisher/trajectoryID", target_trajectoryID_, 0);
  nh_.param<int>("trajectory_publisher/mode", mode_, 2);

  newTraj=false;
  dimension = 3;
  derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  // std::cout<<target_trajectoryID_<<std::endl;
  // std::cout<<target_trajectoryID_<<std::endl;
  std::cout<<target_trajectoryID_<<std::endl;
  std::cout<<mode_<<std::endl;
  
  traj_axis_ << 0.0, 0.0, 1.0;
  p_targ << 0.0, 0.0, 0.0;
  v_targ << 0.0, 0.0, 0.0;
  acc_targ << 0.0, 0.0, 0.0;
  target_initpos << init_pos_x_, init_pos_y_, init_pos_z_;
  
  ros::Duration(10.0).sleep(); // sleep for half a second
  
  //<param name="marker_scale" value="6" type="double"/>
  
  setTrajectory(target_trajectoryID_);
}

void trajectoryPublisher::localCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
  if (std::isfinite(msg.pose.pose.position.x) && std::isfinite(msg.pose.pose.position.y) && std::isfinite(msg.pose.pose.position.z)){
    mavPos_(0) = msg.pose.pose.position.x;
    mavPos_(1) = msg.pose.pose.position.y;
    mavPos_(2) = msg.pose.pose.position.z;
    mavAtt_(0) = msg.pose.pose.orientation.w;
    mavAtt_(1) = msg.pose.pose.orientation.x;
    mavAtt_(2) = msg.pose.pose.orientation.y;	    
    mavAtt_(3) = msg.pose.pose.orientation.z;
  }
  else{
    std::cout<<"xt is not finite!!!"<<std::endl;
  }
}

void trajectoryPublisher::stateCallback(const mavros_msgs::State &msg){
  state = msg;
}

void trajectoryPublisher::setTrajectory(int ID) {
  double radius = 0, omega = 0;
  Eigen::Vector3d axis, initpos;

  switch (ID) {
    case TRAJ_STATIONARY: //stationary trajectory
      omega = 0.0;
      radius = 0.0;
      axis << 0.0, 0.0, 1.0;
      initpos << init_pos_x_, init_pos_y_, init_pos_z_;
      break;
    case TRAJ_CIRCLE: //circular trajectory
      omega = 0.5;//1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, radius, 1.0;
      break;
    case TRAJ_LAMNISCATE: //Lemniscate of Genoro
      omega = 1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, radius, 0.0;
      break;
    case TRAJ_POLY:
      getPolyTrajectory();
      break;      
    case (TRAJ_POLY+1):
      getPolyTrajectory();
      break;    
  }
  // std::cout<<initpos<<", "<<ID<<std::endl;
  // std::cout<<initpos<<", "<<ID<<std::endl;
  // std::cout<<initpos<<", "<<ID<<std::endl;
  setTrajectory(ID, omega, axis, radius, initpos);
}

void trajectoryPublisher::getPolyTrajectory(void){

  ros::Time stime = ros::Time::now();
  /*
  Eigen::Vector3d center(0,0,2);
  // createSquareVertices(int maximum_derivative,
  //                                   const Eigen::Vector3d& center,
  //                                   double side_length, int rounds)
  mav_trajectory_generation::Vertex::Vector vertices = mav_trajectory_generation::createSquareVertices(derivative_to_optimize,center,5,10);
  */

  std::cout<<"trajID: "<<target_trajectoryID_<<std::endl;
  
  mav_trajectory_generation::Vertex::Vector vertices;
  mav_trajectory_generation::Vertex start(dimension), middle(dimension), middle2(dimension), middle3(dimension), middle4(dimension), middle5(dimension), end(dimension);

  start.makeStartOrEnd(Eigen::Vector3d(mavPos_(0),mavPos_(1),mavPos_(2)), derivative_to_optimize);
  vertices.push_back(start);  
    
  if (target_trajectoryID_ == 3){
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1, 1, 1));
    vertices.push_back(middle);

    middle2.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1,1,1));
    vertices.push_back(middle2);

    middle3.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1,-1,1));
    vertices.push_back(middle3);

    middle4.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,-1,1));
    vertices.push_back(middle4);

  }
  else if (target_trajectoryID_ == 4){

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,0,1));
    middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    // middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));
    vertices.push_back(middle);

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,5,1));
    middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    // middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));
    vertices.push_back(middle);

    middle2.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,0,1));
    middle2.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    // middle2.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));
    vertices.push_back(middle2);

    middle3.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,5,1));
    middle3.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    // middle3.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));
    vertices.push_back(middle3);

    middle4.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,0,1));
    middle4.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    // middle4.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));
    vertices.push_back(middle4);    
  }

  end.makeStartOrEnd(Eigen::Vector3d(mavPos_(0),mavPos_(1),1), derivative_to_optimize);
  vertices.push_back(end);

  
  
  // end.makeStartOrEnd(Eigen::Vector3d(0,0,2), derivative_to_optimize);
  // vertices.push_back(end);


  // 3. compute the segment times
  // std::vector<double> segment_times(vertices.size()-1,50);
  std::vector<double> segment_times;
  // double v_max = 10;//6.0;
  // double v_max = 4;
  double v_max = 3;
  double a_max = 2;//2.5;//10;
  // double j_max=9*(counter%2)+1;//10;//10*(counter%2)+1;
  double j_max=3;

  counter++;
  std::cout<<"v_max: "<<v_max<<std::endl;
  std::cout<<"a_max: "<<a_max<<std::endl;
  std::cout<<"j_max: "<<j_max<<std::endl;

  // input_constraints.addConstraint(ICT::kFMax, 1.5 * 9.81); // maximum acceleration in [m/s/s].
  // input_constraints.addConstraint(ICT::kVMax, 5.5); // maximum velocity in [m/s].

  double timeMultiplier=1;
  // segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max*.5, a_max*.5);
  segment_times = mav_trajectory_generation::estimateSegmentTimesNfabian(vertices, v_max*timeMultiplier, a_max*timeMultiplier);

  

  //--- Nonlinear Optimization ---//

  // 1. Set up the problem by following Steps 1-3 in the Linear Optimization section.

  // 2. Set the parameters for nonlinear optimization. Below is an example, but the default parameters should be reasonable enough to use without fine-tuning.
  
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  
  // parameters.max_iterations = 1000;
  // parameters.f_rel = 0.05;
  // parameters.x_rel = 0.1;
  // parameters.time_penalty = 500;//1750*(counter%2)+250;//500.0;
  // parameters.initial_stepsize_rel = 0.1;
  // parameters.inequality_constraint_tolerance = 0.01;

  parameters.max_iterations = 3000;
  parameters.f_rel = 1.0e-6;
  // parameter.f_rel = 0.01;
  // parameter.x_rel = 0.01;
  parameters.time_penalty = 0.00000001;
  parameters.initial_stepsize_rel = .1;//-1;

  parameters.inequality_constraint_tolerance = .01;
  // parameter.soft_constraint_weight = 100;
  parameters.soft_constraint_weight = .1; // timeH sensitive!!
  // parameters.use_soft_constraints = false;

  if (target_trajectoryID_ == 3){
  parameters.time_alloc_method =
    mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTimeAndConstraints;
  }
  else if (target_trajectoryID_ == 4){
    parameters.time_alloc_method =
      mav_trajectory_generation::NonlinearOptimizationParameters::kMellingerOuterLoop;
    parameters.algorithm = nlopt::LD_LBFGS;
    std::fill (segment_times.begin(),segment_times.end(),3);
  }

  // parameters.algorithm = nlopt::LD_LBFGS;
  // parameters.time_alloc_method =
    // mav_trajectory_generation::NonlinearOptimizationParameters::kMellingerOuterLoop;
  
  // 3. Create an optimizer object and solve. The third argument of the optimization object (true/false) specifies whether the optimization is run over the segment times only.

  const int Nnl = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<Nnl> nlOpt(this->dimension, parameters);
  // mav_trajectory_generation::PolynomialOptimizationNonLinear<Nnl> nlOpt(dimension, parameters, false);
  nlOpt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  nlOpt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  nlOpt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
  nlOpt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, j_max);
  nlOpt.optimize();

  std::cout<< nlOpt.getOptimizationInfo() <<std::endl;

  // 4. Obtain the polynomial segments.

  // mav_trajectory_generation::Segment::Vector segments;
  nlOpt.getPolynomialOptimizationRef().getSegments(&segments);

  // check if its feasible

  typedef mav_trajectory_generation::InputConstraintType ICT;
  mav_trajectory_generation::InputConstraints input_constraints;
  input_constraints.addConstraint(ICT::kFMin, 0); // minimum acceleration in [m/s/s].
  input_constraints.addConstraint(ICT::kFMax, a_max+9.81); // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(ICT::kVMax, v_max); // maximum velocity in [m/s].
  input_constraints.addConstraint(ICT::kOmegaXYMax, 0.5); // maximum roll/pitch rates in [rad/s].
  // input_constraints.addConstraint(ICT::kOmegaZMax, M_PI / 2.0); // maximum yaw rates in [rad/s].
  // input_constraints.addConstraint(ICT::kOmegaZDotMax, M_PI); // maximum yaw acceleration in [rad/s/s].

  // Create feasibility object of choice (FeasibilityAnalytic,
  // FeasibilitySampling, FeasibilityRecursive).
  mav_trajectory_generation::FeasibilityAnalytic feasibility_check(input_constraints);
  feasibility_check.settings_.setMinSectionTimeS(0.01);

  // Create feasibility check for ground
  mav_trajectory_generation::FeasibilityBase feasibility_check_ground;
  // Create ground plane.
  Eigen::Vector3d point(0.0, 0.0, 0.0);
  Eigen::Vector3d normal(0.0, 0.0, 1.0);
  feasibility_check_ground.half_plane_constraints_.emplace_back(point, normal);

  int ts = 1;

  // Check feasibility.
  for (int i=0; i<segments.size(); i++){    
    mav_trajectory_generation::InputFeasibilityResult result = feasibility_check.checkInputFeasibility(segments[i]);
    if (result!=0 || !feasibility_check_ground.checkHalfPlaneFeasibility(segments[i])){
      ts=ts-5;
      std::cout << "The segment["<<i<<"] is" << mav_trajectory_generation::getInputFeasibilityResultName(result)<< "." << std::endl;
      std::cout << "The segment["<<i<<"] is in collision with the ground plane." << std::endl;
    }
  }
  std::cout<<"total compute time: "<<ros::Time::now().toSec()-stime.toSec()<<std::endl;
  
  if (ts==1){
    //--- Creating Trajectories ---//
    nlOpt.getTrajectory(&trajectory_poly);
    newTraj=true;

    //For a simple visualization:
    double distance = 2.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    // From Trajectory class:
    mav_trajectory_generation::drawMavTrajectory(trajectory_poly, distance, frame_id, &markers);
    ros::Publisher markers_pub = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory", 1);
    start_time_ = ros::Time::now();
  }
}

void trajectoryPublisher::setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius,
                                        Eigen::Vector3d initpos) {
  target_trajectoryID_ = ID;
  traj_axis_ = axis;
  traj_omega_ = omega;
  traj_radius_ = radius;
  target_initpos = initpos;
}

void trajectoryPublisher::setTrajectoryTheta(double in) {
  theta_ = in;
}

void trajectoryPublisher::moveReference() {
  curr_time_ = ros::Time::now();

  if (state.mode.compare("OFFBOARD")!=0 || !state.armed){
    start_time_ = ros::Time::now();
  }
  
  trigger_time_ = (curr_time_ - start_time_).toSec();

  // std::cout<<"mode: "<<mode_<<std::endl;

  if(mode_ == MODE_PRIMITIVES){
    //TODO: Play reference trajectory based on time
//    p_targ << a0x + a1x*trigger_time_ + a2x*trigger_time_^2 + a3x*trigger)time_^3,
//              a0y + a1y*trigger_time_ + a2y*trigger_time_^2 + a3y*trigger)time_^3,
//              a0z + a1z*trigger_time_ + a2z*trigger_time_^2 + a3z*trigger)time_^3;
//    v_targ << a1 + 2*a2*trigger_time_ + 3*a3*trigger)time_^2,
//              a1 + 2*a2*trigger_time_ + 3*a3*trigger)time_^2,
//              a1 + 2*a2*trigger_time_ + 3*a3*trigger)time_^2;
  }
  else if(mode_ == MODE_REFERENCE){
    theta_ = traj_omega_* trigger_time_;

    // std::cout<<"trajId: "<<target_trajectoryID_<<std::endl;
    if (target_trajectoryID_ == 0) { //Stationary
      p_targ = target_initpos;
      v_targ.setZero();
    } else if (target_trajectoryID_ == 1) { //Circular trajectory
      
      p_targ = std::cos(theta_) * target_initpos
               + std::sin(theta_) * traj_axis_.cross(target_initpos)
               + (1 - std::cos(theta_)) * traj_axis_.dot(target_initpos) * traj_axis_;
      v_targ = traj_omega_ * traj_axis_.cross(p_targ);
    } else if (target_trajectoryID_ == 2) { //Lemniscate of Genero
      p_targ = std::cos(theta_) * target_initpos
               + std::sin(theta_) * std::cos(theta_) * traj_axis_.cross(target_initpos)
               + (1 - std::cos(theta_)) * traj_axis_.dot(target_initpos) * traj_axis_;
      v_targ = traj_omega_ * traj_axis_.cross(p_targ); //TODO: This is wrong      
    } else if ((target_trajectoryID_ == 3 || target_trajectoryID_ == 4) && newTraj) {// polynomial      
        int derivativeP = mav_trajectory_generation::derivative_order::POSITION;
	int derivativeV = mav_trajectory_generation::derivative_order::VELOCITY;
	int derivativeA = mav_trajectory_generation::derivative_order::ACCELERATION;

	p_targ   = trajectory_poly.evaluate(trigger_time_,derivativeP);
	v_targ   = trajectory_poly.evaluate(trigger_time_,derivativeV);
	acc_targ = trajectory_poly.evaluate(trigger_time_,derivativeA);

	if (p_targ.isApprox(v_targ) && v_targ.isApprox(p_targ)){
	  p_targ = target_initpos;
	  v_targ.setZero();
	  acc_targ.setZero();
	  // target_trajectoryID_=0;
	  getPolyTrajectory();
	}
	
	markers_pub.publish(markers);
    }
  }
}

Eigen::Vector3d trajectoryPublisher::getTargetPosition(){
  return p_targ;
}

double trajectoryPublisher::getTrajectoryOmega(){
  return traj_omega_;
}

double trajectoryPublisher::getTrajectoryUpdateRate(){
  return controlUpdate_dt_;
}

geometry_msgs::PoseStamped trajectoryPublisher::vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "world";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

void trajectoryPublisher::pubrefTrajectory(){

int N = (int) 2*3.141592 / this->getTrajectoryOmega() / this->getTrajectoryUpdateRate(); //Resolution of the trajectory to be published
double theta;
Eigen::Vector3d targetPosition;
Eigen::Vector4d targetOrientation;
targetOrientation << 1.0, 0.0, 0.0, 0.0;
geometry_msgs::PoseStamped targetPoseStamped;

refTrajectory_.header.stamp = ros::Time::now();
refTrajectory_.header.frame_id = 1;

for(int i = 0 ; i < N ; i++){
  theta = 2 * 3.141592 *((double) i / (double) N);
  this->setTrajectoryTheta(theta);
  this->moveReference();
  targetPosition = this->getTargetPosition();
  targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
  refTrajectory_.poses.push_back(targetPoseStamped);
}
}

void trajectoryPublisher::pubrefState(){
  refState_.header.stamp = ros::Time::now();
  refState_.header.frame_id = "world";
  refState_.twist.angular.x = p_targ(0);
  refState_.twist.angular.y = p_targ(1);
  refState_.twist.angular.z = p_targ(2);
  refState_.twist.linear.x = v_targ(0);
  refState_.twist.linear.y = v_targ(1);
  refState_.twist.linear.z = v_targ(2);

  // std::cout<<(v_targ.norm()>2.5)<<", "<<v_targ.norm()<<std::endl;
  referencePub_.publish(refState_);
  
  refAccel.header.stamp = ros::Time::now();
  refAccel.header.frame_id = "world";  
  refAccel.accel.linear.x = acc_targ(0);
  refAccel.accel.linear.y = acc_targ(1);
  refAccel.accel.linear.z = acc_targ(2);

  accelReference_pub.publish(refAccel);
}

void trajectoryPublisher::loopCallback(const ros::TimerEvent& event){
  //Slow Loop publishing trajectory information
  nh_.param<int>(agentName+"/quadMode", quadMode, -1);
  trajectoryPub_.publish(refTrajectory_);
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
  moveReference();
  pubrefState();
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){ // this shouldn't be done through Bool j
  unsigned char mode = req.data;
  start_time_ = ros::Time::now();
  //TODO: Trajectory triggering should not be done by changing modes
  switch(mode){
  case 0 :
    target_trajectoryID_ = TRAJ_STATIONARY;      
    break;
  case 1 :
    target_trajectoryID_ = TRAJ_CIRCLE;
    break;
  case 2 :
    target_trajectoryID_ = TRAJ_LAMNISCATE;
    break;
  }

  std::cout<<"trajID:"<<mode<<std::endl;
  nh_.setParam("/trajectory_publisher/trajectoryID", target_trajectoryID_);
  res.success = true;
  res.message = "trajectory triggered";
}

void trajectoryPublisher::trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_msg) {

  start_time_ = ros::Time::now();
  //TODO: implement a trajectory replay functionality
//  segments_message.segments;
//
//  motionPrimitives_.setPolynomial();

}
