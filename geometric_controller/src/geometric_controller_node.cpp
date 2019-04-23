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

  mode=10000;
  nh_.param<int>("geometric_controller/agent_number", AGENT_NUMBER, 1);
  
  /// Target State is the reference state received from the trajectory
  /// goalState is the goal the controller is trying to reach
  // goalPos_ << 2*(AGENT_NUMBER-1), 0.0, 1.5; //Initial Position // needs check so that my quads don't freak out
  
  targetVel_ << 0.0, 0.0, 0.0;
  mavYaw_ = M_PI/2.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -3.0, -3.0, -60.0;
  Kvel_ << -3.0, -3.0, -60.0;
  std::vector<double> kp, kv;

  nh_.getParam("geometric_controller/kp", kp);
  Kpos_<<kp[0],kp[1],kp[2];
  
  nh_.getParam("geometric_controller/kv", kv);
  Kvel_<<kv[0],kv[1],kv[2];


  D_ << 0.0, 0.0, 0.0;
  // attctrl_tau_ = 0.15; //0.2;
  // attctrl_tau_ << 0.20, 0.20, .2;
  attctrl_tau_.resize(3);
  attctrl_tau_[0] = 0.3;
  attctrl_tau_[1] = 0.3;
  attctrl_tau_[2] = 0.3;
  nh_.getParam("geometric_controller/attctrl_tau_", attctrl_tau_);

  nh_.param<double>("geometric_controller/norm_thrust_const_", norm_thrust_const_, 0.1);
  nh_.param<double>("geometric_controller/max_fb_acc_", max_fb_acc_, 5.0);

  use_gzstates_ = false;

  agentName = ros::this_node::getNamespace();  
  agentName.erase(0,1);

  targetPos_history.resize(25);
  targetVel_history.resize(25);

  errorVel_history.resize(50);
  ev_idx=0;

  a_des_history.resize(10);
  ades_idx=0;

  std::fill(a_des_history.begin(), a_des_history.end(), Eigen::Vector3d(0,0,0));

  std::fill(errorVel_history.begin(), errorVel_history.end(), Eigen::Vector3d(0,0,0));

  std::fill(targetPos_history.begin(), targetPos_history.end(), Eigen::Vector3d(0,0,0));
  std::fill(targetVel_history.begin(), targetVel_history.end(), Eigen::Vector3d(0,0,0));

  avoiding=false;
  timeFlag=false;
  finishedAvoid_time=ros::Time::now();
  // avoid_time=ros::Time::now();

  tpvh=0;
  
  nh_.param<string>("geometric_controller/mav_name", mav_name_, "iris");
  nh_.param<int>("geometric_controller/ctrl_mode", ctrl_mode_, MODE_BODYRATE);
  nh_.param<bool>("geometric_controller/enable_sim", sim_enable_, true);
  nh_.param<bool>("geometric_controller/tuneRate", tuneRate, false);
  nh_.param<bool>("geometric_controller/tuneAtt", tuneAtt, false); 
  nh_.getParam("geometric_controller/desiredRate", desiredRate);
  nh_.getParam("geometric_controller/desiredAtt", desiredAtt);


  nh_.param<int>(agentName+"/pf/agents", numAgents, 4);  
  nh_.param<float>("geometric_controller/radius", radius, 2.5);
  nh_.param<bool>("geometric_controller/obstaclesOn", obstaclesOn, false);

  newRefData=false;
  newDataFlag=false;
  newVelData.resize(numAgents);
  newPosData.resize(numAgents);
  std::fill(newVelData.begin(), newVelData.end(), false);
  std::fill(newPosData.begin(), newPosData.end(), false);

  agentInfo_time.resize(numAgents);
  std::fill(agentInfo_time.begin(), agentInfo_time.end(), ros::Time::now());

  xt.set_size(numAgents,3); // xyz  
  set_all_elements(xt,0);

  vt.set_size(numAgents,3); // xyz  
  set_all_elements(vt,0);  

  nh_.param<double>("/v_max", v_max, 3);
  nh_.param<bool>("/avoidAgents", avoidAgents, true);

  rcSub_ = nh_.subscribe(agentName+"/mavros/rc/in",1,&geometricCtrl::rc_command_callback,this,ros::TransportHints().tcpNoDelay());
  referenceSub_=nh_.subscribe(agentName+"/reference/setpoint",1, &geometricCtrl::targetCallback,this,ros::TransportHints().tcpNoDelay());

  accelReferenceSub_=nh_.subscribe(agentName+"/reference/accel",1, &geometricCtrl::targetAccelCallback,this,ros::TransportHints().tcpNoDelay());
    
  flatreferenceSub_ = nh_.subscribe(agentName+"/reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
  mavstateSub_ = nh_.subscribe(agentName+"/mavros/state", 1, &geometricCtrl::mavstateCallback, this,ros::TransportHints().tcpNoDelay());

  if(tuneAtt || tuneRate){
    mavposeSub_ = nh_.subscribe(agentName+"/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  }
  else{  
    mavposeSub_ = nh_.subscribe(agentName+"/local_position", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
    // mavposeSub_ = nh_.subscribe(agentName+"/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  }

  mavtwistSub_ = nh_.subscribe(agentName+"/mavros/local_position/velocity", 1, &geometricCtrl::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
   // mavtwistSub_ = nh_.subscribe(agentName+"/local_velocity", 1, &geometricCtrl::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  
  gzmavposeSub_ = nh_.subscribe("/gazebo/model_states", 1, &geometricCtrl::gzmavposeCallback, this, ros::TransportHints().tcpNoDelay());

  agentSub_ = nh_.subscribe("/agent_mps_data",1, &geometricCtrl::agentsCallback, this, ros::TransportHints().tcpNoDelay());

    
  ctrltriggerServ_ = nh_.advertiseService(agentName+"/tigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback, this); // Define timer for constant loop rate

  for(int i=0;i<numAgents;++i){    
    std::string topicName = "agent"+std::to_string(AGENT_NUMBER)+"_"+std::to_string(i+1);
    agentPos_pub.push_back(nh_.advertise<visualization_msgs::Marker>(topicName, 1));

  }
  for(int i=0;i<numAgents;++i){    
    std::string topicName = "agentVel"+std::to_string(AGENT_NUMBER)+"_"+std::to_string(i+1);
    agentVel_pub.push_back(nh_.advertise<visualization_msgs::Marker>(topicName, 1));

  }
  
  bPub_ = nh_.advertise<geometry_msgs::TwistStamped>(agentName+"/b", 1);
  obstaclesPub_ = nh_.advertise<visualization_msgs::Marker>(agentName+"/obstacles",1);
  
  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(agentName+"/command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>(agentName+"/reference/pose", 1);

  // if (tuneAtt){
  des_attRefPub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(agentName+"/desAtt",1);
  cur_attRefPub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(agentName+"/curAtt",1);
  error_attPub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(agentName+"/errorAtt",1);

  mavPosVelPub_ = nh_.advertise<geometry_msgs::TwistStamped>(agentName+"/mavPosVel",1);
  mavAccelPub_  = nh_.advertise<geometry_msgs::AccelStamped>(agentName+"/mavAccel",1);
  // }

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(agentName+"/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(agentName+"/mavros/set_mode");
  
  frame_client    = nh_.serviceClient<mavros_msgs::SetMavFrame>(agentName+"/mavros/mav_frame");
  
  mav_frame.request.mav_frame = mav_frame.request.FRAME_BODY_NED;
  std::cout<<"frame call: "<<frame_client.call(mav_frame)<<std::endl;

  if (tuneRate && tuneAtt){
    ROS_ERROR("tuneRate=true and tuneAtt=true, only tune one");
    while(ros::ok()){ros::Duration(20.0).sleep();}
  }

  sim = new RVO::RVOSimulator();
    
  wait4Home(); // ensures I have gotten positions from other agents at least
    
  setupScenario();

  targetPos_ << xt(AGENT_NUMBER-1,0), xt(AGENT_NUMBER-1,1), 1;
  
  // ros::Duration(15.0).sleep(); // sleep for half a second
}
geometricCtrl::~geometricCtrl() {
  delete sim;
  //Destructor
}


void geometricCtrl::updateAgents(void) {

  updateGoal();
  for(int i=0; i<numAgents; i++){
    if (i==(AGENT_NUMBER-1)){

      RVO::Vector2 desVel;

      desVel = 2*(RVO::Vector2(targetPos_noCA(0), targetPos_noCA(1)) - RVO::Vector2(mavPos_(0), mavPos_(1))) + 0.5*RVO::Vector2(targetVel_noCA(0), targetVel_noCA(1));

      sim->setAgentPrefVelocity(i, desVel);
      // sim->setAgentPrefVelocity(i, RVO::Vector2(targetVel_noCA(0), targetVel_noCA(1)));      
      // sim->setAgentPrefVelocity(i, RVO::Vector2(mavVel_(0), mavVel_(1)));

      sim->setAgentPosition    (i, RVO::Vector2(mavPos_(0), mavPos_(1)));
      sim->setAgentVelocity    (i, RVO::Vector2(mavVel_(0), mavVel_(1)));
    }
    else{      
      sim->setAgentPrefVelocity(i, RVO::Vector2(vt(i,0), vt(i,1)));
      sim->setAgentPosition    (i, RVO::Vector2(xt(i,0), xt(i,1)));
      sim->setAgentVelocity    (i, RVO::Vector2(vt(i,0), vt(i,1)));
      
    }
  }
}

void geometricCtrl::updateCA_velpos(void){
  updateAgents();
  sim->doStep();
  // sim->doStep();  // predict farther forward?
  RVO::Vector2 pos, vel;

  // std::cout<<"vel: "<<std::endl;
  // std::cout<<vt<<std::endl;
  
  pos = sim->getAgentPosition(AGENT_NUMBER-1);
  vel = sim->getAgentVelocity(AGENT_NUMBER-1);

  // check for runaway agents
  // double dir = std::atan2(vel.y(), vel.x());
  // double othDir;

  // int runAway=0;  
  // std::vector<int> runAwayAgent;

  // for(int i=0; i<numAgents; i++){
  //   if (i!=(AGENT_NUMBER-1)){
  //     othDir = std::atan2(vt(i,1), vt(i,0));

  //     // std::cout<<"angle diff: "<<fabs(othDir - dir)*180/M_PI<<std::endl;
  //     // std::cout<<"pos diff: "<<dlib::length(rowm(xt, AGENT_NUMBER) - rowm(xt, i))<<std::endl;
  //     // std::cout<<"xt: "<<xt<<std::endl;
  //     // std::cout<<"sub: "<<(rowm(xt, AGENT_NUMBER) - rowm(xt, i))<<std::endl;

  //     if ((fabs(othDir - dir)-M_PI)<0.5 && (mavPos_-targetPos_noCA).norm()>3){	//dlib::length(rowm(xt, AGENT_NUMBER) - rowm(xt, i))<(2.0*radius)
  // 	runAwayAgent.push_back(i+1);
  // 	runAway+=1;
  //     }
  //   }    
  // }
  
  Eigen::Vector3d targetPos_CA, targetVel_CA;

  targetVel_CA << vel.x(), vel.y(), targetVel_noCA(2);
  if (targetVel_CA.norm()>0.05){
    targetPos_CA << pos.x(), pos.y(), targetPos_noCA(2);
  }
  else{
    //this fixes drift at the end of traj
    targetPos_CA=targetPos_noCA;
    targetVel_CA=targetVel_CA*0;
  }
  
  
  // double b;// = (targetVel_CA-targetVel_noCA).norm();
  // double bx = fabs(targetVel_CA(0)-targetVel_noCA(0));
  // double by = fabs(targetVel_CA(1)-targetVel_noCA(1));

  // double comp = 0.01*AGENT_NUMBER;
  
  // // if (b>.005){ // avoid
  // if (bx>comp || by>comp){ // avoid if any difference in traj
  //   b=1;
  //   targetPos_ = targetPos_CA;
  //   targetVel_ = targetVel_CA;    
  //   avoiding=true;
  //   timeFlag=false;

  //   // if (!runAwayAgent.empty()){
  //   //   for (int i=0; i<runAwayAgent.size(); i++){	
  //   // 	if (runAwayAgent[i]>AGENT_NUMBER){
  //   // 	  targetVel_ = targetVel_*(AGENT_NUMBER*0.1f);
  //   // 	}
  //   //   }
  //   // }
    
  //   // avoid_time=ros::Time::now();
    
  // }
  // // else if(b<.005 && avoiding){
  // else if((bx<comp && by<comp) && avoiding){ // consider stop avoiding when both xy traj are similar to original
  //   //if i was avoiding and about to stop avoiding
    
  //   if(!timeFlag){//if i havent' started timing
  //     finishedAvoid_time=ros::Time::now(); // start timeing
  //     timeFlag=true;
  //   }
    
  //   if((ros::Time::now()-finishedAvoid_time).toSec()>2 && mavVel_.norm()>0.25){
  //     // make sure to wait for three seconds until i follow original pos traj
  //     avoiding=false;
  //     targetPos_ = targetPos_noCA;
  //     targetVel_ = targetVel_noCA;
  //     b=0;
  //     timeFlag=false;
  //   }
  //   else{ // continue to avoid 
  //     targetPos_ = targetPos_CA;
  //     targetVel_ = targetVel_CA;
  //     b=0.5;
  //   }
  // }
  // else{ // no need to avoid
  //   b=0;
  //   targetPos_ = targetPos_noCA;
  //   targetVel_ = targetVel_noCA;
  //   avoiding=false;
  //   timeFlag=false;
  // }


  targetPos_ = targetPos_CA;
  targetVel_ = targetVel_CA;

  /*

  targetPos_history[tpvh] = targetPos_;
  targetVel_history[tpvh] = targetVel_;

  // double b = b_sigMoid((targetVel_CA-targetVel_noCA).norm(), 100, .01);

  // targetPos_ = std::accumulate(targetPos_history.begin(), targetPos_history.end(), Eigen::Vector3d(0,0,0)) / targetPos_history.size();
  // targetVel_ = std::accumulate(targetVel_history.begin(), targetVel_history.end(), Eigen::Vector3d(0,0,0)) / targetVel_history.size();

  targetPos_ << 0,0,0;
  targetVel_ << 0,0,0;
  for(int i=0;i<targetPos_history.size();i++){
    targetPos_ = targetPos_ + targetPos_history[i];
    targetVel_ = targetVel_ + targetVel_history[i];

    // std::cout<<"i: "<<i<<std::endl;
    // std::cout<<"targetPosHistory[i]: "<<targetPos_history[i]<<std::endl;
    // std::cout<<"targetVelHistory[i]: "<<targetVel_history[i]<<std::endl;
    // std::cout<<"targetPos: "<<targetPos_<<std::endl;    
    // std::cout<<"targetVel: "<<targetVel_<<std::endl;
  }
  targetPos_ = targetPos_/targetPos_history.size();
  targetVel_ = targetVel_/targetVel_history.size();


  // std::cout<<"AVE targetPos: "<<targetPos_<<std::endl;    
  // std::cout<<"AVE targetVel: "<<targetVel_<<std::endl;


  tpvh=tpvh+1;

  if(tpvh>targetPos_history.size()){
    tpvh=0;
  }
  */


  visualization_msgs::Marker otherAgentMarker, otherVelMarker;  
  otherAgentMarker.header.frame_id = "/world";
  otherAgentMarker.header.stamp = ros::Time::now();
  otherAgentMarker.ns = "otherAgentMarker";
  otherAgentMarker.id = 0;


  for(int i=0; i<numAgents; i++){

    visualization_msgs::Marker otherVelMarker;  

    otherVelMarker.header.frame_id = "/world";
    otherVelMarker.header.stamp = ros::Time::now();
    otherVelMarker.ns = "otherVelMarker";
    otherVelMarker.id = 0;


    otherAgentMarker.type = visualization_msgs::Marker::SPHERE;
    otherVelMarker.type = visualization_msgs::Marker::ARROW;

    
    otherVelMarker.action = visualization_msgs::Marker::ADD;
    otherVelMarker.color.a = 1.0;

    
    // tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, targetCenters_mean(i,5));
    // otherAgentMarker.pose.orientation = tf::createQuaternionMsgFromYaw(targetCenters_mean(i,5));
    
    otherAgentMarker.pose.orientation.x = 0.0;
    otherAgentMarker.pose.orientation.y = 0.0;
    otherAgentMarker.pose.orientation.z = 0.0;
    otherAgentMarker.pose.orientation.w = 1.0;
    otherAgentMarker.scale.x = 2*2;// 0.25*5;
    otherAgentMarker.scale.y = 2*2; //0.25*5;
    otherAgentMarker.scale.z = 2*2; //0.25*5;
    otherAgentMarker.pose.position.x = xt(i,0);
    otherAgentMarker.pose.position.y = xt(i,1);
    otherAgentMarker.pose.position.z = xt(i,2);

    otherAgentMarker.action = visualization_msgs::Marker::ADD;
    otherAgentMarker.color.a = .25;

    geometry_msgs::Point p1,p2;

    p1.x = xt(i,0);
    p1.y = xt(i,1);
    p1.z = xt(i,2);
    
    p2.x = xt(i,0)+vt(i,0);
    p2.y = xt(i,1)+vt(i,1);
    p2.z = xt(i,2);
    // p2.x = vt(i,0);
    // p2.y = vt(i,1);


    otherVelMarker.scale.x = 0.25;
    otherVelMarker.scale.y = 0.25;
    otherVelMarker.scale.z = 0.25;

    otherVelMarker.points.push_back(p1);
    otherVelMarker.points.push_back(p2);

    if (AGENT_NUMBER == 1)
      {

	// if (b>=1 && AGENT_NUMBER!=i){
	//   otherAgentMarker.color.r = 1.0;
	//   otherAgentMarker.color.g = 0.0;
	//   otherAgentMarker.color.b = 0.0;
	// }
	// else{
	  otherAgentMarker.color.r = 0.0;
	  otherAgentMarker.color.g = 1.0;
	  otherAgentMarker.color.b = 0.0;
	// }

	otherVelMarker.color.r = 0.0;
	otherVelMarker.color.g = 1.0;
	otherVelMarker.color.b = 0.0;

      }
    else if (AGENT_NUMBER == 2)
      {
	
	// if (b>=1 && AGENT_NUMBER!=i){
	//   otherAgentMarker.color.r = 1.0;
	//   otherAgentMarker.color.g = 0.0;
	//   otherAgentMarker.color.b = 0.0;
	// }
	// else{
	  otherAgentMarker.color.r = 0.0;
	  otherAgentMarker.color.g = 0.0;
	  otherAgentMarker.color.b = 1.0;
	// }

	otherVelMarker.color.r = 0.0;
	otherVelMarker.color.g = 0.0;
	otherVelMarker.color.b = 1.0;

      }
    else if (AGENT_NUMBER == 3)
      {
	
	// if (b>=1 && AGENT_NUMBER!=i){
	//   otherAgentMarker.color.r = 1.0;
	//   otherAgentMarker.color.g = 0.0;
	//   otherAgentMarker.color.b = 0.0;
	// }
	// else{
	  otherAgentMarker.color.r = 1.0;
	  otherAgentMarker.color.g = 0.0;
	  otherAgentMarker.color.b = 1.0;
	// }

	otherVelMarker.color.r = 1.0;
	otherVelMarker.color.g = 0.0;
	otherVelMarker.color.b = 1.0;

      }
    else if (AGENT_NUMBER == 4)
      {
	  otherAgentMarker.color.r = 0.0;
	  otherAgentMarker.color.g = 1.0;
	  otherAgentMarker.color.b = 1.0;
	  
	  otherVelMarker.color.r = 0.0;
	  otherVelMarker.color.g = 1.0;
	  otherVelMarker.color.b = 1.0;

      }
    else if (AGENT_NUMBER == 5)
      {
	  otherAgentMarker.color.r = 1.0;
	  otherAgentMarker.color.g = 0.5;
	  otherAgentMarker.color.b = 0.0;
	  
	  otherVelMarker.color.r = 1.0;
	  otherVelMarker.color.g = 0.5;
	  otherVelMarker.color.b = 0.0;

      }
    else{
      otherAgentMarker.color.r = 1.0;
      otherAgentMarker.color.g = 1.0;
      otherAgentMarker.color.b = 0.0;
	  
      otherVelMarker.color.r = 1.0;
      otherVelMarker.color.g = 1.0;
      otherVelMarker.color.b = 0.0;
    }
      


      agentPos_pub[i].publish(otherAgentMarker);
      agentVel_pub[i].publish(otherVelMarker);
  }

  


    
  // b_msg.header.stamp = ros::Time::now();
  // b_msg.twist.linear.x = b;
  // // b_msg.twist.linear.y = runAway;
  // bPub_.publish(b_msg);

  
  geometry_msgs::TwistStamped mavposvel;

  mavposvel.header.stamp=ros::Time::now();
  mavposvel.twist.angular.x = targetPos_(0);
  mavposvel.twist.angular.y = targetPos_(1);
  mavposvel.twist.angular.z = targetPos_(2);

  mavposvel.twist.linear.x = targetVel_(0);
  mavposvel.twist.linear.y = targetVel_(1);
  mavposvel.twist.linear.z = targetVel_(2);

  mavPosVelPub_.publish(mavposvel);

}

double geometricCtrl::b_sigMoid(double x, double c, double a){
  return 1/(1+std::exp(-c*(x-a)));
}

void geometricCtrl::wait4Home(void){
    // Wait until GPS reference parameters are initialized.
  do {
    // ROS_INFO("[C] Waiting for GPS reference parameters...");
    if (nh_.getParam("/gps_ref_latitude", H_latitude) &&
        nh_.getParam("/gps_ref_longitude", H_longitude) &&
        nh_.getParam("/gps_ref_altitude", H_altitude)){
      g_geodetic_converter.initialiseReference(H_latitude, H_longitude, H_altitude);
    }
    
    nh_.param<std::string>("/runAlg", runAlg, "lawnMower");    
    ros::Duration(0.01).sleep();
    ros::spinOnce();

    if(tuneAtt || tuneRate){
      break;
    }

    ROS_INFO_THROTTLE(5,"[%d ctrl] waiting for pos and vel data from other agents and reference data %d, %d, %d, %d", AGENT_NUMBER, !g_geodetic_converter.isInitialised(), std::any_of(newPosData.begin(),newPosData.end(), [](bool v) {return !v;}), std::any_of(newVelData.begin(),newVelData.end(), [](bool v) {return !v;}), !newRefData);
    
  } while ((!g_geodetic_converter.isInitialised() || std::any_of(newPosData.begin(),newPosData.end(), [](bool v) {return !v;}) || std::any_of(newVelData.begin(),newVelData.end(), [](bool v) {return !v;}) || !newRefData || (runAlg.compare("info")!=0)) && ros::ok()); // wait until i have home and i have recied pos, vel data from all agents.
  //} while ((!g_geodetic_converter.isInitialised() || !newRefData) && ros::ok()); // wait until i have home and i have recied pos, vel data from all agents.

  newDataFlag=true;
}

void geometricCtrl::updateGoal(void){

  if (!goals.empty()){    
    goals.clear();
  }
  // Create goals (simulator is unaware of these).
  for (int i = 0; i < numAgents; ++i) {
    if (i==(AGENT_NUMBER-1)){
      // goals.push_back(RVO::Vector2(targetPos_(0),targetPos_(1)));
      goals.push_back(RVO::Vector2(targetPos_noCA(0),targetPos_noCA(1)));
    }
    else{
      goals.push_back(RVO::Vector2(xt(i,0)+vt(i,0)*.01,xt(i,1)+vt(i,1)*.01));  // set goal ahead
    }
  }

}


void geometricCtrl::setupScenario(void) {
  //wait4Home ensures I have necessary variables set.
  
  sim->setTimeStep(.01f);
  
  // neighborDist,maxNeighbors,timeHorizon,timeHorizonObst,radius,maxSpeed,    
  sim->setAgentDefaults(30.0f, numAgents*2, 5.0f, 2.5f, radius, 1.5*v_max);

  for (int i=0;i<numAgents; i++){
    sim->addAgent(RVO::Vector2(xt(i,0), xt(i,1)));
  }

  updateGoal();

  // for (size_t i = 0; i < sim->getNumAgents(); ++i) {
  //   std::cout<<"goals: "<<goals[i]<<std::endl;    
  // }

  // // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
  std::vector<std::vector<RVO::Vector2>> obstacles;
  // obstacles.resize(3);

  std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

  // must be counterclockwise or will fail!!!
  obstacle1.push_back(RVO::Vector2(0.0f, 15.0f));
  obstacle1.push_back(RVO::Vector2(5.0f, 25.0f));
  obstacle1.push_back(RVO::Vector2(-5.0f, 25.0f));

  obstacles.push_back(obstacle1);

  
  // obstacle2.push_back(RVO::Vector2(-20.0f, 5.0f));
  // obstacle2.push_back(RVO::Vector2(-25.0f, -5.0f));
  // obstacle2.push_back(RVO::Vector2(-15.0f, -5.0f));

  // obstacles.push_back(obstacle2);

  
  obstacle3.push_back(RVO::Vector2(0.0f, -15.0f));
  obstacle3.push_back(RVO::Vector2(-5.0f, -25.0f));  
  obstacle3.push_back(RVO::Vector2(5.0f, -25.0f));
  
  obstacles.push_back(obstacle3);

  
  visualization_msgs::Marker obstacleMsg;
  obstacleMsg.header.frame_id = "/world";
  obstacleMsg.header.stamp = ros::Time::now();
  obstacleMsg.ns = "obstacles";
  obstacleMsg.id = 0;
  
  obstacleMsg.type = visualization_msgs::Marker::LINE_LIST; // this can be move to constructor
  obstacleMsg.pose.orientation.x = 0.0;
  obstacleMsg.pose.orientation.y = 0.0;
  obstacleMsg.pose.orientation.z = 0.0;
  obstacleMsg.pose.orientation.w = 1.0;
  obstacleMsg.scale.x = 0.25*1;
  std_msgs::ColorRGBA clr;
  clr.r = 1;
  clr.g = 0;
  clr.b = 0;
  clr.a = 1.0;

  for (int j=0; j<obstacles.size(); j++){
    for (int i=0; i<obstacles[j].size(); i++){
      geometry_msgs::Point p;
      if(i!=obstacles[j].size()-1){
  	p.x=obstacles[j][i].x();
  	p.y=obstacles[j][i].y();
  	obstacleMsg.points.push_back(p);
  	obstacleMsg.colors.push_back(clr);

  	p.x=obstacles[j][i+1].x();
  	p.y=obstacles[j][i+1].y();
  	obstacleMsg.points.push_back(p);
  	obstacleMsg.colors.push_back(clr);
      }
      else{
  	p.x=obstacles[j][i].x();
  	p.y=obstacles[j][i].y();
  	obstacleMsg.points.push_back(p);
  	obstacleMsg.colors.push_back(clr);

  	p.x=obstacles[j][0].x();
  	p.y=obstacles[j][0].y();
  	obstacleMsg.points.push_back(p);
  	obstacleMsg.colors.push_back(clr);
      }
    }
  }


  if (obstaclesOn){
    obstaclesPub_.publish(obstacleMsg);
    
    for (int i=0; i<obstacles.size(); i++){
      sim->addObstacle(obstacles[i]);
    }
    
    sim->processObstacles();
  }
}

void geometricCtrl::agentsCallback(const enif_iuc::AgentMPS &msg){ // slow rate

  bool validGPS = msg.mps.GPS_latitude<180 && msg.mps.GPS_latitude>-180 && msg.mps.GPS_longitude<180 && msg.mps.GPS_longitude>-180 && (msg.mps.GPS_latitude!=0 && msg.mps.GPS_longitude!=0);
    
  if (validGPS && msg.agent_number!=AGENT_NUMBER && g_geodetic_converter.isInitialised())
  {
    // std::cout<<agentName<<" got other agent data"<<std::endl;
    // ROS_INFO("got agent %d 's data: ", msg.agent_number, msg.mps.percentLEL);
    

    double x,y,z;

    // convert to local ENU frame
    g_geodetic_converter.geodetic2Enu(msg.mps.GPS_latitude, msg.mps.GPS_longitude, H_altitude, &xt(msg.agent_number-1,0), &xt(msg.agent_number-1,1), &xt(msg.agent_number-1,2));    
    xt(msg.agent_number-1,2) = msg.mps.local_z;

    // g_geodetic_converter.geodetic2Enu(msg.mps.GPS_latitude, msg.mps.GPS_longitude, H_altitude, &x,&y,&z);
    // z = msg.mps.local_z;

 
   
    // xt(msg.agent_number-1,0) = .75*xt(msg.agent_number-1,0) + .25*x;
    // xt(msg.agent_number-1,1) = .75*xt(msg.agent_number-1,1) + .25*y;
    // xt(msg.agent_number-1,2) = .75*xt(msg.agent_number-1,2) + .25*z;
    
    vt(msg.agent_number-1,0) = msg.mps.vel_x;
    vt(msg.agent_number-1,1) = msg.mps.vel_y;
    vt(msg.agent_number-1,2) = msg.mps.vel_z;

    // vt(msg.agent_number-1,0) = .75*vt(msg.agent_number-1,0) + .25*msg.mps.vel_x;
    // vt(msg.agent_number-1,1) = .75*vt(msg.agent_number-1,1) + .25*msg.mps.vel_y;
    // vt(msg.agent_number-1,2) = .75*vt(msg.agent_number-1,2) + .25*msg.mps.vel_z;
    
    xt(msg.agent_number-1,0) = xt(msg.agent_number-1,0)+vt(msg.agent_number-1,0)*(ros::Time::now()-agentInfo_time[msg.agent_number-1]).toSec();
    xt(msg.agent_number-1,1) = xt(msg.agent_number-1,1)+vt(msg.agent_number-1,1)*(ros::Time::now()-agentInfo_time[msg.agent_number-1]).toSec();
    xt(msg.agent_number-1,2) = xt(msg.agent_number-1,2)+vt(msg.agent_number-1,2)*(ros::Time::now()-agentInfo_time[msg.agent_number-1]).toSec();

    // std::cout<<"AGENT_NUMBER: "<<AGENT_NUMBER<<std::endl;
    // std::cout<<"msg.agent_number: "<<msg.agent_number<<std::endl;
    // std::cout<<"xt: "<<xt<<std::endl;
    // std::cout<<"vt: "<<vt<<std::endl;

    newPosData[msg.agent_number-1]=true;
    newVelData[msg.agent_number-1]=true;

    agentInfo_time[msg.agent_number-1] = ros::Time::now();
  }
  else{
    int t=1;
    ros::spinOnce();
  }
}


void geometricCtrl::rc_command_callback(const mavros_msgs::RCIn::ConstPtr &new_message)
{
  RCin = *new_message;
  mode = RCin.channels[4];
}


void geometricCtrl::targetAccelCallback(const geometry_msgs::AccelStamped& msg) {
  targetAcc_ << msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z;
  newRefData=true;

  accel_CA.header.stamp = ros::Time::now();
  accel_CA.accel.linear.x=targetAcc_(0);
  accel_CA.accel.linear.y=targetAcc_(1);
  accel_CA.accel.linear.z=targetAcc_(2);

  mavAccelPub_.publish(accel_CA);

}
void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped& msg) {

  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  targetPos_noCA_prev_ = targetPos_noCA;
  targetVel_noCA_prev_ = targetVel_noCA;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_noCA << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
  targetVel_noCA << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;

  // Eigen::Vector3d pos,vel;
  targetPos_ << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
  targetVel_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;

  if (avoidAgents && newDataFlag){
    updateCA_velpos();
  }
  
  
  if (targetVel_.norm()>=0.25){
    mavYaw_ = std::atan2(targetVel_(1),targetVel_(0));
  }
  else{
    mavYaw_ = tf::getYaw(tf::Quaternion(mavAtt_(1), mavAtt_(2), mavAtt_(3), mavAtt_(0)));
  }

  mavYaw_ = M_PI/2;
  
 
  // if(reference_request_dt_ > 0 && targetVel_.norm()>=0.25 && false) targetAcc_ = 0.9*targetAcc_ + 0.1*(targetVel_ - targetVel_prev_ ) / reference_request_dt_; // I can get this from trajectory!!!
  
  // if(reference_request_dt_ > 0 && targetVel_noCA.norm()>=0.25) targetAcc_ = (targetVel_noCA - targetVel_noCA_prev_ ) / reference_request_dt_; // I can get this from trajectory!!!
  // else targetAcc_ = Eigen::Vector3d::Zero();
  

  // accel_CA.header.stamp = ros::Time::now();
  // accel_CA.accel.linear.x=targetAcc_(0);
  // accel_CA.accel.linear.y=targetAcc_(1);
  // accel_CA.accel.linear.z=targetAcc_(2);

  // mavAccelPub_.publish(accel_CA);
  

  newRefData=true;

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

    //    /*
    if (tuneAtt){ 
      tf::Quaternion qc(mavAtt_(1), mavAtt_(2), mavAtt_(3), mavAtt_(0)), qnew;
      
      tf::Matrix3x3 mc(qc);      
      double rollC, pitchC, yawC;
      mc.getRPY(rollC, pitchC, yawC);
      //mavYaw_=yawC;
      mc.setRPY(rollC,pitchC,mavYaw_); // SET the yaw to be desired!!!
      mc.getRotation(qnew);      
      mavAtt_(0) = qnew.w();
      mavAtt_(1) = qnew.x();
      mavAtt_(2) = qnew.y();
      mavAtt_(3) = qnew.z();
    }
    //    */
    
    newPosData[AGENT_NUMBER-1]=true;
    xt(AGENT_NUMBER-1,0) = mavPos_(0);
    xt(AGENT_NUMBER-1,1) = mavPos_(1);
    xt(AGENT_NUMBER-1,2) = mavPos_(2);
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
    newVelData[AGENT_NUMBER-1]=true;
    vt(AGENT_NUMBER-1,0) = mavVel_(0);
    vt(AGENT_NUMBER-1,1) = mavVel_(1);;
    vt(AGENT_NUMBER-1,2) = mavVel_(2);
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
	newPosData[AGENT_NUMBER-1]=true;
      }
    }
  }
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent& event){
  
  // if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
  //   offb_set_mode_.request.custom_mode = "OFFBOARD";
  //   if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
  //     ROS_INFO("Offboard enabled");
  //   }
  //   last_request_ = ros::Time::now();
  // }
  
  nh_.param<std::string>("/runAlg", runAlg, "lawnMower");
  
  if (runAlg.compare("info")==0 && newDataFlag){
  if(!sim_enable_){
    if(mode<1100 && (tuneAtt || tuneRate)){
      // Enable OFFBoard mode and arm automatically
      arm_cmd_.request.value = true;      
      if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
	if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
	  ROS_INFO("Vehicle armed");
	}
	last_request_ = ros::Time::now();
      }
    }
    else if(mode>1100 && (tuneAtt || tuneRate)){
      ROS_INFO_THROTTLE(5,"flip remote switch");
	arm_cmd_.request.value = false;
	if( current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
	  if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
	    ROS_INFO("Vehicle disarmed");
	  }
	  last_request_ = ros::Time::now();
	}
    }
    //else{
      // if i am not tuning then let client handle arming and offboard mode
    //}
    

  }
  else{
    arm_cmd_.request.value = true;      
    if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
	ROS_INFO("Vehicle armed");
      }
      last_request_ = ros::Time::now();
    }
    
    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      offb_set_mode_.request.custom_mode = "OFFBOARD";
      if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
	ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
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
  //ros::spinOnce();
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void geometricCtrl::statusloopCallback(const ros::TimerEvent& event){

}

void geometricCtrl::pubReferencePose(){
  referencePoseMsg_.header.stamp = ros::Time::now();
  referencePoseMsg_.header.frame_id = "world";
  referencePoseMsg_.pose.position.x = targetPos_noCA(0);
  referencePoseMsg_.pose.position.y = targetPos_noCA(1);
  referencePoseMsg_.pose.position.z = targetPos_noCA(2);
  // referencePoseMsg_.pose.position.x = targetPos_(0);
  // referencePoseMsg_.pose.position.y = targetPos_(1);
  // referencePoseMsg_.pose.position.z = targetPos_(2);

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
  if (tuneRate){
    ROS_INFO_ONCE("tuning rate");
    nh_.getParam("geometric_controller/desiredRate", desiredRate);
    ROS_INFO_THROTTLE(3,"desiredRate: wx=%f, wy=%f, wz=%f, t=%f", desiredRate[0], desiredRate[1], desiredRate[2], desiredRate[3]);
    cmdBodyRate_(0) = desiredRate[0]; // roll
    cmdBodyRate_(1) = desiredRate[1]; // pitch
    cmdBodyRate_(2) = desiredRate[2]; // yaw
    cmdBodyRate_(3) = desiredRate[3]; // thrust
  }
  else if(tuneAtt){
    ROS_INFO_ONCE("tuning att");
    nh_.getParam("geometric_controller/desiredAtt", desiredAtt);
    nh_.getParam("geometric_controller/attctrl_tau_", attctrl_tau_);
    ROS_INFO_THROTTLE(3,"attctrl_tau_=[%f, %f, %f] desiredAtt: [%f, %f, %f]", attctrl_tau_[0], attctrl_tau_[1], attctrl_tau_[2], desiredAtt[0], desiredAtt[1], desiredAtt[2]);
    
    a_ref(0) = desiredAtt[0];
    a_ref(1) = desiredAtt[1];
    a_ref(2) = desiredAtt[2];

    a_ref = 2.0*a_ref.normalized(); // prevent large accelerations
        
    a_des = a_ref;// - g_;
    q_des = acc2quaternion(a_des, mavYaw_);

    geometry_msgs::QuaternionStamped qdes, qcur;
    qdes.header.stamp = ros::Time::now();
    qcur.header.stamp = qdes.header.stamp;
        
    qdes.quaternion.x = q_des(1);
    qdes.quaternion.y = q_des(2);
    qdes.quaternion.z = q_des(3);
    qdes.quaternion.w = q_des(0);
    
    qcur.quaternion.x = mavAtt_(1);
    qcur.quaternion.y = mavAtt_(2);
    qcur.quaternion.z = mavAtt_(3);
    qcur.quaternion.w = mavAtt_(0);    


    tf::Quaternion qd(q_des(1), q_des(2), q_des(3), q_des(0));      
    tf::Matrix3x3 md(qd);
    double rollD, pitchD, yawD;
    md.getRPY(rollD, pitchD, yawD);

    tf::Quaternion qc(mavAtt_(1), mavAtt_(2), mavAtt_(3), mavAtt_(0));
    tf::Matrix3x3 mc(qc);
    double rollC, pitchC, yawC;
    mc.getRPY(rollC, pitchC, yawC);

    qdes.quaternion.x = rollD*180.0/M_PI; 
    qdes.quaternion.y = pitchD*180.0/M_PI;
    qdes.quaternion.z = yawD*180.0/M_PI;

    qcur.header.stamp=ros::Time::now();
    qcur.quaternion.x=rollC*180.0/M_PI;
    qcur.quaternion.y=pitchC*180.0/M_PI;
    qcur.quaternion.z=yawC*180.0/M_PI;

    des_attRefPub_.publish(qdes);
    cur_attRefPub_.publish(qcur);
                
    cmdBodyRate_ = attcontroller(q_des, a_des, mavAtt_); //Calculate BodyRate

  }
  else{
    Eigen::Vector3d errorPos_, errorVel_, errorPos_noCA, errorVel_filtered;
    Eigen::Matrix3d R_ref;

    errorPos_   = mavPos_ - targetPos_;
    errorVel_   = mavVel_ - targetVel_;

    // errorVel_history[ev_idx]=errorVel_;

    // errorVel_filtered = std::accumulate(errorVel_history.begin(), errorVel_history.end(), Eigen::Vector3d(0,0,0)) / errorVel_history.size();
    
        
    a_ref = targetAcc_;

    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    q_ref = acc2quaternion(a_ref - g_, mavYaw_);
    R_ref = quat2RotMatrix(q_ref);
    a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_; //feedforward term for trajectory error
    // if(a_fb(2) < -max_fb_acc_) a_fb(2) = -max_fb_acc_;
    if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;    
    
    a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * targetVel_; //Rotor drag
    // a_des = a_fb + a_ref - a_rd - g_;
    a_des = a_fb + a_ref - g_;


    // a_des_history[ades_idx]=a_des;    

    // a_des_filtered = std::accumulate(a_des_history.begin(), a_des_history.end(), Eigen::Vector3d(0,0,0)) / a_des_history.size();

    // ades_idx++;

    // if(ades_idx>a_des_history.size()) ades_idx=0;
    
    q_des = acc2quaternion(a_des, mavYaw_);
    cmdBodyRate_ = attcontroller(q_des, a_des, mavAtt_); //Calculate BodyRate

    // ev_idx++;    
    // if (ev_idx>errorVel_history.size()) ev_idx=0;    
    
    // b_msg.header.stamp = ros::Time::now();
    // b_msg.twist.linear.x = errorVel_(0);
    // b_msg.twist.linear.y = errorVel_(1);
    // b_msg.twist.linear.z = errorVel_(2);
    // b_msg.twist.angular.x = errorPos_(0);
    // b_msg.twist.angular.y = errorPos_(1);
    // b_msg.twist.angular.z = errorPos_(2);

    // b_msg.twist.linear.x = Kpos_(0)*errorPos_(0);
    // b_msg.twist.linear.y = Kpos_(1)*errorPos_(1);
    // b_msg.twist.linear.z = Kpos_(2)*errorPos_(2);

    // b_msg.twist.linear.x = Kvel_(0)*errorVel_(0);
    // b_msg.twist.linear.y = Kvel_(1)*errorVel_(1);
    // b_msg.twist.linear.z = Kvel_(2)*errorVel_(2);

    // b_msg.twist.linear.x = a_des_filtered(0);
    // b_msg.twist.linear.y = a_des_filtered(1);
    // b_msg.twist.linear.z = a_des_filtered(2);

    // b_msg.twist.linear.x = Kvel_(0)*errorVel_filtered(0);
    // b_msg.twist.linear.y = Kvel_(1)*errorVel_filtered(1);
    // b_msg.twist.linear.z = Kvel_(2)*errorVel_filtered(2);

    // b_msg.twist.angular.x = Kvel_(0)*errorVel_(0);
    // b_msg.twist.angular.y = Kvel_(1)*errorVel_(1);
    // b_msg.twist.angular.z = Kvel_(2)*errorVel_(2);

    // b_msg.twist.angular.x = a_des(0);
    // b_msg.twist.angular.y = a_des(1);
    // b_msg.twist.angular.z = a_des(2);


    geometry_msgs::QuaternionStamped qdes, qcur;
    qdes.header.stamp = ros::Time::now();
    qcur.header.stamp = qdes.header.stamp;
    qdes.quaternion.x = q_des(1);
    qdes.quaternion.y = q_des(2);
    qdes.quaternion.z = q_des(3);
    qdes.quaternion.w = q_des(0);
    
    qcur.quaternion.x = mavAtt_(1);
    qcur.quaternion.y = mavAtt_(2);
    qcur.quaternion.z = mavAtt_(3);
    qcur.quaternion.w = mavAtt_(0);

    des_attRefPub_.publish(qdes);
    cur_attRefPub_.publish(qcur);
    
    /*
    // convert to current and desired euler rpy and publish both
    tf::Quaternion qd(q_des(1), q_des(2), q_des(3), q_des(0));      
    tf::Matrix3x3 md(qd);
    double rollD, pitchD, yawD;
    md.getRPY(rollD, pitchD, yawD);

    tf::Quaternion qc(mavAtt_(1), mavAtt_(2), mavAtt_(3), mavAtt_(0));
    tf::Matrix3x3 mc(qc);
    double rollC, pitchC, yawC;
    mc.getRPY(rollC, pitchC, yawC);

    des_attRefMsg_.header.stamp = ros::Time::now();
    des_attRefMsg_.vector.x = rollD*180.0/M_PI; 
    des_attRefMsg_.vector.y = pitchD*180.0/M_PI;
    des_attRefMsg_.vector.z = yawD*180.0/M_PI;
    des_attRefPub_.publish(des_attRefMsg_);

    cur_attRefMsg_.header.stamp=ros::Time::now();
    cur_attRefMsg_.vector.x=rollC*180.0/M_PI;
    cur_attRefMsg_.vector.y=pitchC*180.0/M_PI;
    cur_attRefMsg_.vector.z=yawC*180.0/M_PI;
    cur_attRefPub_.publish(cur_attRefMsg_);
    */

    // std::cout<<"pos:"<<std::endl;
    // std::cout<<errorPos_[2]<<std::endl;
    // std::cout<<"vel:"<<std::endl;
    // std::cout<<errorVel_<<std::endl;
    // std::cout<<"acc:"<<std::endl;
    // std::cout<<a_ref<<std::endl;  
  

  }

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
  
  geometry_msgs::QuaternionStamped errorQ;
  errorQ.header.stamp= ros::Time::now();
  errorQ.quaternion.w = qe(0);
  errorQ.quaternion.x = qe(1);
  errorQ.quaternion.y = qe(2);
  errorQ.quaternion.z = qe(3);
  
  error_attPub_.publish(errorQ);
      
  ratecmd(0) = (2.0 / attctrl_tau_[0]) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_[1]) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_[2]) * std::copysign(1.0, qe(0)) * qe(3);
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);
  // ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb))); //Calculate thrust
  ratecmd(3) = std::max(0.0, norm_thrust_const_ * ref_acc.dot(zb)); //Calculate thrust
  // ratecmd(3) = norm_thrust_const_ * ref_acc.dot(zb); //Calculate thrust
  return ratecmd;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
  return true;
}
