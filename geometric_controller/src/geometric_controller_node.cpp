//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "geometric_controller/geometric_controller.h"

#include <chrono>

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
  nh_.param<bool>("geometric_controller/tunePosVel", tunePosVel, false);
  nh_.param<int>("trajectory_publisher/trajectoryID", target_trajectoryID_, -1);
  nh_.param<double>("geometric_controller/sourceObjSize", sourceObjSize, 0.5);
  nh_.param<double>("geometric_controller/gainCA", gainCA, 2.25);

  std::cout<<"gainCA "<<gainCA<<std::endl;
  /// Target State is the reference state received from the trajectory
  /// goalState is the goal the controller is trying to reach
  // goalPos_ << 2*(AGENT_NUMBER-1), 0.0, 1.5; //Initial Position // needs check so that my quads don't freak out

  quadMode.data=1;
  targetVel_ << 0.0, 0.0, 0.0;
  mavYaw_ = M_PI/2.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -3.0, -3.0, -60.0;
  Kvel_ << -3.0, -3.0, -60.0;
  Kint_ << -3.0, -3.0, -60.0;
  inverse << 1.0, -1.0, -1.0, -1.0;
  std::vector<double> kp, kv, ki;

  transCtr=0;
  start=std::chrono::system_clock::now();
  
  newSourceData=false;
  q_des<< -0.7071068,0,0,-0.7071068;
  holdPos_<<0,0,0;

  nh_.getParam("geometric_controller/kp", kp);
  Kpos_<<kp[0],kp[1],kp[2];
  
  nh_.getParam("geometric_controller/kv", kv);
  Kvel_<<kv[0],kv[1],kv[2];

  nh_.getParam("geometric_controller/ki", ki);  
  Kint_<<ki[0],ki[1],ki[2];

  nh_.param<double>("geometric_controller/takeOffThrust", takeOffThrust, .60);
  nh_.param<bool>("geometric_controller/outerBox", outerBox, false);
  nh_.param<double>("geometric_controller/xyAccelMax", xyAccelMax, 3.0);

  sumAtt <<0,0,0;
  
  D_ << 0.0, 0.0, 0.0;
  errorSum_ << 0.0, 0.0, 0.0;
  errorsumOri_ = RVO::Vector2(0.0,0.0);
  
  attctrl_tau_p.resize(3);
  attctrl_tau_d.resize(3);
  attctrl_tau_i.resize(3);
  
  nh_.getParam("geometric_controller/attctrl_tau_p", attctrl_tau_p);
  nh_.getParam("geometric_controller/attctrl_tau_i", attctrl_tau_i);
  nh_.getParam("geometric_controller/attctrl_tau_d", attctrl_tau_d);

  nh_.param<double>("geometric_controller/max_tau_i", max_tau_i, 0.5);
  nh_.param<double>("geometric_controller/norm_thrust_const_", norm_thrust_const_, 0.1);
  nh_.param<double>("geometric_controller/max_fb_acc_", max_fb_acc_, 5.0);
  nh_.param<double>("geometric_controller/max_rollRate", max_rollRate, 10);
  nh_.param<double>("geometric_controller/max_pitchRate", max_pitchRate, 10);
  nh_.param<double>("geometric_controller/max_yawRate", max_yawRate, 10);


  use_gzstates_ = false;

  agentName = ros::this_node::getNamespace();  
  agentName.erase(0,1);

  targetPos_history.resize(25);
  targetVel_history.resize(25);

  errorVel_history.resize(50);
  ev_idx=0;

  a_des_history.resize(30);
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
  nh_.param<float>("geometric_controller/timeH", timeH, 5);
  nh_.param<bool>("geometric_controller/obstaclesOn", obstaclesOn, false);

  nh_.param<int>("trajectory_publisher/trajectoryID", target_trajectoryID_, -1);
  if (target_trajectoryID_==4){
    obstaclesOn=true;
    ROS_INFO("[GEO] trajId=4, avoiding static obstacles!!!");
  }

  nh_.param<bool>("/useKalman", useKalman, true);

  ctrs.resize(numAgents);  
  startTimes.resize(numAgents);
  e_time.resize(numAgents);
  std::fill(ctrs.begin(), ctrs.end(), 0);
  
  if (useKalman){

    // Discrete 2D integrator dynamics, measuring position and velocity 
    n=4;
    m=4;
    kl_dt_x=0.01;
  
    A.resize(n,n); // System dynamics matrix
    C.resize(m,n); // Output matrix
    Q.resize(n,n); // Process noise covariance
    R.resize(m,m); // Measurement noise covariance  
    P.resize(n,n); // Estimate error covariance
    
    A << 1, kl_dt_x ,0,       0,
         0,       1 ,0,       0,
         0,       0, 1, kl_dt_x,
         0,       0, 0,       1;
  
    C = MatrixXd::Identity(m, n);  // all states

      // Reasonable covariance matrices  
    Q << 0.1, 0.0,   0,   0,
         0.0, 0.01,   0,   0,
           0,   0, 0.1, 0.0,
           0,   0, 0.0, 0.01;
    Q=kl_dt_x*Q;      
    
    R <<   0.05, 0.0,   0,   0,
           0.0, 0.05,   0,   0,
             0,   0, 0.05, 0.0,
             0,   0, 0.0, 0.05;
    
    P << 5, 0,  0, 0,
         0,  1,  0, 0,
         0,  0, 5, 0,
         0,  0,  0, 1;

    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;

  }

  kfInit.resize(numAgents);
  std::fill(kfInit.begin(), kfInit.end(), false);
  kfs.resize(numAgents);

  newRefData=false;
  newDataFlag=false;
  simSetup=false;
  newVelData.resize(numAgents);
  newPosData.resize(numAgents);
  std::fill(newVelData.begin(), newVelData.end(), false);
  std::fill(newPosData.begin(), newPosData.end(), false);

   agentInfo_time.resize(numAgents);
   std::fill(agentInfo_time.begin(), agentInfo_time.end(), ros::Time::now());

   xt.set_size(numAgents,3); // xyz  
   set_all_elements(xt,0);

   if (tunePosVel){
     for(int i=0;i<numAgents; i++){
       if(i!=(AGENT_NUMBER-1)){
	 xt(i,0)= 1000;
	 xt(i,1)= 1000;
	 xt(i,2)= 1000;
       }
     }
   }

   vt.set_size(numAgents,3); // xyz  
   set_all_elements(vt,0);  

   nh_.param<double>("/v_max", v_max, 2);
   nh_.param<bool>("/avoidAgents", avoidAgents, true);

   source_sub = nh_.subscribe("/agent_source_data",1,&geometricCtrl::sourceCallback,this,ros::TransportHints().tcpNoDelay());

  rcSub_ = nh_.subscribe(agentName+"/mavros/rc/in",1,&geometricCtrl::rc_command_callback,this,ros::TransportHints().tcpNoDelay());
  referenceSub_=nh_.subscribe(agentName+"/reference/setpoint",1, &geometricCtrl::targetCallback,this,ros::TransportHints().tcpNoDelay());

  accelReferenceSub_=nh_.subscribe(agentName+"/reference/accel",1, &geometricCtrl::targetAccelCallback,this,ros::TransportHints().tcpNoDelay());
    
   flatreferenceSub_ = nh_.subscribe(agentName+"/reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
   mavstateSub_ = nh_.subscribe(agentName+"/mavros/state", 1, &geometricCtrl::mavstateCallback, this,ros::TransportHints().tcpNoDelay());

   if(tuneAtt || tuneRate){
     mavposeSub_ = nh_.subscribe(agentName+"/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
   }
   else{  
     // mavposeSub_ = nh_.subscribe(agentName+"/local_position", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
     local_sub   = nh_.subscribe(agentName+"/gps_pose", 1, &geometricCtrl::localCallback, this, ros::TransportHints().tcpNoDelay()); // from geodetic
     // mavposeSub_ = nh_.subscribe(agentName+"/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
   }

   mavtwistSub_ = nh_.subscribe(agentName+"/mavros/local_position/velocity", 1, &geometricCtrl::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
    // mavtwistSub_ = nh_.subscribe(agentName+"/local_velocity", 1, &geometricCtrl::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());

  
   gzmavposeSub_ = nh_.subscribe("/gazebo/model_states", 1, &geometricCtrl::gzmavposeCallback, this, ros::TransportHints().tcpNoDelay());


   agentSub_ = nh_.subscribe("/agent_mps_data",1, &geometricCtrl::agentsCallback, this, ros::TransportHints().tcpNoDelay());

   ctrltriggerServ_ = nh_.advertiseService(agentName+"/tigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
   cmdloop_timer_    = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback, this); 
   statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback, this); 
   arming_timer_     = nh_.createTimer(ros::Duration(.2), &geometricCtrl::armingCallback, this);    
   checkData_timer_  = nh_.createTimer(ros::Duration(5), &geometricCtrl::checkDataCallback, this);

   quadModePub_ = nh_.advertise<std_msgs::Int16>(agentName+"/quadMode", 1);
   hzPub_ = nh_.advertise<std_msgs::Float64MultiArray>(agentName+"/otherQuadHz", 1);
  
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
   controlActionPub_ = nh_.advertise<geometry_msgs::TwistStamped>(agentName+"/controlAction",1);
   controlActionIntPub_ = nh_.advertise<geometry_msgs::TwistStamped>(agentName+"/controlActionInt",1);
 
   
   angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(agentName+"/command/bodyrate_command", 1);
  referencePosePubCA_ = nh_.advertise<geometry_msgs::PoseStamped>(agentName+"/reference/poseCA", 1);
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

   wait4Home(); // ensures I have gotten positions from other agents at least

   if (!tuneRate && !tuneAtt){
     setupScenario();
   }
   targetPos_ << xt(AGENT_NUMBER-1,0), xt(AGENT_NUMBER-1,1), 1;
 }
geometricCtrl::~geometricCtrl() {
   delete sim;
   //Destructor
 }


void geometricCtrl::updateAgents(void) {

  updateGoal();
  dlib::matrix<double> disp;
  double k=2*radius*gainCA;
  double ktime=1;

  std_msgs::Float64MultiArray quadHz;
  std_msgs::MultiArrayDimension myDim;
  
  quadHz.layout.dim.push_back(myDim);
  quadHz.layout.dim[0].size = numAgents;
  quadHz.layout.dim[0].stride = 2;
  
  for(int i=0; i<numAgents; i++){
    if (i==(AGENT_NUMBER-1)){

      //errorsumOri_ = errorsumOri_+(RVO::Vector2(targetPos_noCA(0), targetPos_noCA(1)) - RVO::Vector2(mavPos_(0), mavPos_(1)));
      
      // if (AGENT_NUMBER==1){
      // 	// here!
      // 	targetVel_noCA<<0,0,0;
      // }

      desVel = 2*(RVO::Vector2(targetPos_noCA(0), targetPos_noCA(1)) - RVO::Vector2(mavPos_(0), mavPos_(1))) + 0.5*RVO::Vector2(targetVel_noCA(0), targetVel_noCA(1));
      
      //desVel = 3*(RVO::Vector2(targetPos_noCA(0), targetPos_noCA(1)) - RVO::Vector2(mavPos_(0), mavPos_(1))) + 1/3.0*(RVO::Vector2(targetVel_noCA(0), targetVel_noCA(1))- RVO::Vector2(mavVel_(0),mavVel_(1)));      
      //desVel = RVO::Vector2(targetVel_noCA(0), targetVel_noCA(1));

      sim->setAgentPrefVelocity(i, desVel);
      // sim->setAgentPrefVelocity(i, RVO::Vector2(targetVel_noCA(0), targetVel_noCA(1)));      
      // sim->setAgentPrefVelocity(i, RVO::Vector2(mavVel_(0), mavVel_(1)));

      sim->setAgentPosition    (i, RVO::Vector2(mavPos_(0), mavPos_(1)));
      sim->setAgentVelocity    (i, RVO::Vector2(mavVel_(0), mavVel_(1)));
    }
    else{
      disp = -rowm(xt,i)+rowm(xt,AGENT_NUMBER-1);

      if(ctrs[i]!=0){
	e_time[i] = std::chrono::system_clock::now()-startTimes[i];

	if (e_time[i].count() < 0.25){
	  ktime=1;
	}
	else{
	  ktime = 4*e_time[i].count();
	}
	// if ((e_time[i].count()/ctrs[i])<0.1){
	//   ktime=1;
	// }
	// else{
	//   ktime=(e_time[i].count()/ctrs[i])/0.1;
	// }
      }


      quadHz.data.push_back(e_time[i].count());
      quadHz.data.push_back(ktime);      

      //std::cout<<"ktime["<<i<<"]="<<ktime<<std::endl;

      if (mavVel_.norm()>1){      
	disp = k/dlib::length(disp)*disp/dlib::length(disp)*mavVel_.norm()*ktime;
      }
      else{
	disp = k/dlib::length(disp)*disp/dlib::length(disp)*ktime;
      }
                        
      // sim->setAgentPrefVelocity(i, RVO::Vector2(vt(i,0), vt(i,1)));
      // sim->setAgentVelocity    (i, RVO::Vector2(vt(i,0), vt(i,1)));
      
      sim->setAgentPrefVelocity(i, RVO::Vector2(vt(i,0)+disp(0), vt(i,1)+disp(1)));
      sim->setAgentPosition    (i, RVO::Vector2(xt(i,0), xt(i,1)));
      sim->setAgentVelocity    (i, RVO::Vector2(vt(i,0)+disp(0), vt(i,1)+disp(1)));
      
    }
  }
  
  hzPub_.publish(quadHz);
}

void geometricCtrl::updateCA_velpos(void){
  
  if (transCtr==0){ // start timer
    start = std::chrono::system_clock::now();
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds=end-start;
    
  //std::cout<<"transmit time="<<t1<<std::endl;
  transCtr++;
  //std::cout<<"transmit per sec "<<transCtr<<", "<<elapsed_seconds.count()<<", "<<transCtr/(elapsed_seconds.count())<<std::endl;

  kl_dt_x = elapsed_seconds.count()/transCtr;

  sim->setTimeStep(kl_dt_x); 

  //std::cout<<"kl_dt_x="<<kl_dt_x<<std::endl;


  
  if (useKalman){
    
    A << 1, kl_dt_x ,0,       0,
      0,       1 ,0,       0,
      0,       0, 1, kl_dt_x,
      0,       0, 0,       1;
    
    Eigen::VectorXd state(m);
    for (int i=0; i<numAgents; i++){   
      if (i!=(AGENT_NUMBER-1) && kfInit[i]){
	kfs[i].set_dt(kl_dt_x, A);
	kfs[i].predict();
	state<<kfs[i].state();
	
	xt(i,0)=state[0];
	vt(i,0)=state[1];
	xt(i,1)=state[2];
	vt(i,1)=state[3];

      }
    }
  }  
  
  updateAgents();
  sim->doStep();
  // sim->doStep();  // predict farther forward?
  RVO::Vector2 pos, vel;

  // std::cout<<"vel: "<<std::endl;
  // std::cout<<vt<<std::endl;
  
  pos = sim->getAgentPosition(AGENT_NUMBER-1);
  vel = sim->getAgentVelocity(AGENT_NUMBER-1);
  
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
  
  targetPos_ = targetPos_CA;
  targetVel_ = targetVel_CA;


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
    otherAgentMarker.scale.x = radius*2;//2*2;// 0.25*5;
    otherAgentMarker.scale.y = radius*2;//2*2; //0.25*5;
    otherAgentMarker.scale.z = radius*2;//2*2; //0.25*5;
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


void geometricCtrl::checkDataCallback(const ros::TimerEvent& event){
  
  std::vector<double> kp, kv, ki;
  nh_.getParam("geometric_controller/kp", kp);
  Kpos_<<kp[0],kp[1],kp[2];
  
  nh_.getParam("geometric_controller/kv", kv);
  Kvel_<<kv[0],kv[1],kv[2];

  nh_.getParam("geometric_controller/ki", ki);
  Kint_<<ki[0],ki[1],ki[2];

  nh_.param<double>("geometric_controller/norm_thrust_const_", norm_thrust_const_, 0.1);

  nh_.getParam("geometric_controller/attctrl_tau_p", attctrl_tau_p);
  nh_.getParam("geometric_controller/attctrl_tau_i", attctrl_tau_i);
  nh_.getParam("geometric_controller/attctrl_tau_d", attctrl_tau_d);

  if (nh_.getParam("/gps_ref_latitude", H_latitude) &&
        nh_.getParam("/gps_ref_longitude", H_longitude) &&
        nh_.getParam("/gps_ref_altitude", H_altitude)){
      g_geodetic_converter.initialiseReference(H_latitude, H_longitude, H_altitude);
  }    
  nh_.param<std::string>("/runAlg", runAlg, "lawnMower");
  
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if(tuneAtt || tuneRate){
      break;
    }

    if (tunePosVel && newVelData[AGENT_NUMBER-1] && newPosData[AGENT_NUMBER-1] && newRefData && g_geodetic_converter.isInitialised() && runAlg.compare("info")==0){
      break;
    }
    else if (tunePosVel){
      ROS_INFO_THROTTLE(5,"[%d ctrl] tunePosVel checks: geodetic_init=%d, newPosData=%d, newVelData=%d, newRefData=%d, runAlg=%d", AGENT_NUMBER, !g_geodetic_converter.isInitialised(), newPosData[AGENT_NUMBER-1], newVelData[AGENT_NUMBER-1], newRefData, (runAlg.compare("info")==0));
    }
    else{
      ROS_INFO_THROTTLE(5,"[%d ctrl] checks: geodetic_init=%d, newPosData=%d, newVelData=%d, newRefData=%d, runAlg=%d", AGENT_NUMBER, !g_geodetic_converter.isInitialised(), std::any_of(newPosData.begin(),newPosData.end(), [](bool v) {return !v;}), std::any_of(newVelData.begin(),newVelData.end(), [](bool v) {return !v;}), !newRefData, (runAlg.compare("info")!=0));
	
    }
    // ROS_INFO_THROTTLE(5,"[%d ctrl] checks: geodetic_init=%d, newPosData=%d, newVelData=%d, newRefData=%d, runAlg=%d, newSourceData=%d", AGENT_NUMBER, !g_geodetic_converter.isInitialised(), std::any_of(newPosData.begin(),newPosData.end(), [](bool v) {return !v;}), std::any_of(newVelData.begin(),newVelData.end(), [](bool v) {return !v;}), !newRefData, (runAlg.compare("info")!=0), !newSourceData);
    

  // } while ((!g_geodetic_converter.isInitialised() || std::any_of(newPosData.begin(),newPosData.end(), [](bool v) {return !v;}) || std::any_of(newVelData.begin(),newVelData.end(), [](bool v) {return !v;}) || !newRefData || (runAlg.compare("info")!=0) || (!newSourceData || !sim_enable_)) && ros::ok()); // wait until i have home and i have recied pos, vel data from all agents.
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

      // if (AGENT_NUMBER==1){
      // 	// here!
      // 	targetPos_noCA=holdPos_;
      // }
      
      goals.push_back(RVO::Vector2(targetPos_noCA(0),targetPos_noCA(1)));
    }
    else{
      goals.push_back(RVO::Vector2(xt(i,0)+vt(i,0)*.01,xt(i,1)+vt(i,1)*.01));  // set goal ahead
    }
  }

}


void geometricCtrl::setupScenario(void) {
  //wait4Home ensures I have necessary variables set.
  sim = new RVO::RVOSimulator();
  
  sim->setTimeStep(.01f); 
  
  // neighborDist,maxNeighbors,timeHorizon,timeHorizonObst,radius,maxSpeed,
  sim->setAgentDefaults(30.0f, numAgents*2, timeH, 1.0f, radius, 1.25*v_max);
  

  for (int i=0;i<numAgents; i++){
    sim->addAgent(RVO::Vector2(xt(i,0), xt(i,1)));
  }

  updateGoal();
  // for (size_t i = 0; i < sim->getNumAgents(); ++i) {
  //   std::cout<<"goals: "<<goals[i]<<std::endl;    
  // }

  // // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
  std::vector<std::vector<RVO::Vector2>> obstacles;
  std::vector<RVO::Vector2> obstacle;
  // obstacles.resize(3);

  if (newSourceData){
    obstacle.push_back(RVO::Vector2(xs,ys+sourceObjSize/2.0));
    obstacle.push_back(RVO::Vector2(xs-sourceObjSize/2.0, ys-sourceObjSize/2.0));
    obstacle.push_back(RVO::Vector2(xs+sourceObjSize/2.0, ys-sourceObjSize/2.0));

    obstaclesOn=true;
    
    obstacles.push_back(obstacle);
    // obstacle.clear();
    ROS_INFO("[geo] placing obstacle over x=%f, y=%f", xs, ys);
  }

  if (target_trajectoryID_==4){

    float triSize=0.25;
    double x, y;
    x=1;
    y=4;

    obstacle.push_back(RVO::Vector2(x,y+triSize/2.0));
    obstacle.push_back(RVO::Vector2(x-triSize/2.0, y-triSize/2.0));
    obstacle.push_back(RVO::Vector2(x+triSize/2.0, y-triSize/2.0));
    
    obstacles.push_back(obstacle);
    obstacle.clear();
  }
  

  if (outerBox){
    std::vector<double> tNoise; // target noise assumption
    nh_.getParam(agentName+"/pf/t_noise", tNoise);
    int targetStates = tNoise.size(); // number of states
    
    SS.set_size(targetStates,2);
    
    nh_.setParam("/boxXmin", SS(0,0));
    nh_.setParam("/boxXmax", SS(0,1));    
    nh_.setParam("/boxYmin", SS(1,0));
    nh_.setParam("/boxYmax", SS(1,1));
    
    std::vector<double> minT, maxT;
    nh_.getParam(agentName+"/pf/minT", minT);
    nh_.getParam(agentName+"/pf/maxT", maxT);
    
    dlib::matrix<double> tp0, tp1;
    tp0.set_size(targetStates,1);
    tp1.set_size(targetStates,1); 
    // dlib::matrix<double,targetStates,1> tp0, tp1;
    
    // =    x    ,    y   ,      z     ,  Q ,  V ,       A          , Dy,     Dz or tau
    tp0 = minT[0],minT[1],minT[2],minT[3],minT[4],minT[5]*M_PI/180.0,minT[6],minT[7];    
    tp1 = maxT[0],maxT[1],maxT[2],maxT[3],maxT[4],maxT[5]*M_PI/180.0,maxT[6],maxT[7];
    
    dlib::set_colm(SS,0)=tp0;
    dlib::set_colm(SS,1)=tp1;
    
    // std::vector<RVO::Vector2> obstacle;
    int buffer=5;
    int width=1;
    
    // counterclockwise:
    
    // bottom
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer, SS(1,0)-buffer));
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer, SS(1,0)-buffer-width));
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer, SS(1,0)-buffer-width));
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer, SS(1,0)-buffer));
    
    obstacles.push_back(obstacle);
    obstacle.clear();
    
    // left
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer, SS(1,0)-buffer));
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer, SS(1,1)+buffer));
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer-width, SS(1,1)+buffer));
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer-width, SS(1,0)-buffer));
    
    obstacles.push_back(obstacle);
    obstacle.clear();
    
    // right
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer, SS(1,0)-buffer));
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer, SS(1,1)+buffer));
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer+width, SS(1,1)+buffer));
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer+width, SS(1,0)-buffer));
    
    obstacles.push_back(obstacle);
    obstacle.clear();
    
    // top
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer, SS(1,1)+buffer));
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer, SS(1,1)+buffer));
    obstacle.push_back(RVO::Vector2(SS(0,1)+buffer, SS(1,1)+buffer+width));
    obstacle.push_back(RVO::Vector2(SS(0,0)-buffer, SS(1,1)+buffer+width));
    
    obstacles.push_back(obstacle);
  }
  
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
  if (target_trajectoryID_==4 || newSourceData){
    obstacleMsg.scale.x = 0.05;
  }
  
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

    ROS_INFO("[geo] processing %f obstacles", obstacles.size());
    sim->processObstacles();
  }

  simSetup=true;
}

void geometricCtrl::agentsCallback(const enif_iuc::AgentMPS &msg){ // slow rate

  bool validGPS = msg.mps.GPS_latitude<180 && msg.mps.GPS_latitude>-180 && msg.mps.GPS_longitude<180 && msg.mps.GPS_longitude>-180 && (msg.mps.GPS_latitude!=0 && msg.mps.GPS_longitude!=0);
    
  if (validGPS && msg.agent_number!=AGENT_NUMBER && g_geodetic_converter.isInitialised())
  {
    //std::cout<<agentName<<" got other agent data"<<std::endl;
    //ROS_INFO("got agent %d 's data: ", msg.agent_number, msg.mps.percentLEL);
    
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

    if ((ros::Time::now()-agentInfo_time[msg.agent_number-1]).toSec() < .2){
      xt(msg.agent_number-1,0) = xt(msg.agent_number-1,0)+vt(msg.agent_number-1,0)*(ros::Time::now()-agentInfo_time[msg.agent_number-1]).toSec();
      xt(msg.agent_number-1,1) = xt(msg.agent_number-1,1)+vt(msg.agent_number-1,1)*(ros::Time::now()-agentInfo_time[msg.agent_number-1]).toSec();
      xt(msg.agent_number-1,2) = xt(msg.agent_number-1,2)+vt(msg.agent_number-1,2)*(ros::Time::now()-agentInfo_time[msg.agent_number-1]).toSec();
    }

    

    if (ctrs[msg.agent_number-1]==0){ // start timer
      startTimes[msg.agent_number-1] = std::chrono::system_clock::now();
      ctrs[msg.agent_number-1]++;
    }
    else{
      e_time[msg.agent_number-1] = std::chrono::system_clock::now()-startTimes[msg.agent_number-1];      
      startTimes[msg.agent_number-1] = std::chrono::system_clock::now();
    }
        
    if (useKalman && kfInit[msg.agent_number-1]){
    
      Eigen::Vector4d Z;
      Z<<xt(msg.agent_number-1,0), vt(msg.agent_number-1,0), xt(msg.agent_number-1,1), vt(msg.agent_number-1,1);
      
      kfs[msg.agent_number-1].update(Z);
            
    }

    if(kfInit[msg.agent_number-1]==false && useKalman){
      Eigen::VectorXd x0(n);
    
      x0 << xt(msg.agent_number-1,0), vt(msg.agent_number-1,0), xt(msg.agent_number-1,1), vt(msg.agent_number-1,1);
      KalmanFilter kf(kl_dt_x, A, C, Q, R, P);
      kf.init(0,x0);
      kfs[msg.agent_number-1]=kf;
      
      kfInit[msg.agent_number-1]=true;
    }

    newPosData[msg.agent_number-1]=true;
    newVelData[msg.agent_number-1]=true;

    agentInfo_time[msg.agent_number-1] = ros::Time::now();
  }
  //else{
  //int t=1;
  //ros::spinOnce();
  //}

}

void geometricCtrl::sourceCallback(const enif_iuc::AgentSource &msg){
  // receive source location from enif_iuc_quad
  if(g_geodetic_converter.isInitialised()){
    g_geodetic_converter.geodetic2Enu(msg.source.latitude, msg.source.longitude, H_altitude, &xs, &ys, &zs);
    // if different then setupScenario:
    ROS_INFO("[geo] got source data: (%f,%f,%f)", xs, ys, zs);    
    newSourceData = true;
    simSetup=false;
    setupScenario();
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

  accel_CA.accel.angular.x=a_des(0);
  accel_CA.accel.angular.y=a_des(1);
  accel_CA.accel.angular.z=a_des(2);

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

  if (avoidAgents && newDataFlag && !tuneAtt && !tuneRate && simSetup){
    updateCA_velpos();
  }  
  
  if (targetVel_.norm()>=0.25){
    mavYaw_ = std::atan2(targetVel_(1),targetVel_(0));
  }
  else{
    mavYaw_ = tf::getYaw(tf::Quaternion(mavAtt_(1), mavAtt_(2), mavAtt_(3), mavAtt_(0)));
  }

  mavYaw_ = M_PI/2;
  
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

void geometricCtrl::localCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
  
  if (std::isfinite(msg.pose.pose.position.x) && std::isfinite(msg.pose.pose.position.y) && std::isfinite(msg.pose.pose.position.z)){
    mavPos_(0) = msg.pose.pose.position.x;
    mavPos_(1) = msg.pose.pose.position.y;
    mavPos_(2) = msg.pose.pose.position.z;
    mavAtt_(0) = msg.pose.pose.orientation.w;
    mavAtt_(1) = msg.pose.pose.orientation.x;
    mavAtt_(2) = msg.pose.pose.orientation.y;	    
    mavAtt_(3) = msg.pose.pose.orientation.z;

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
  else{
    std::cout<<"xt is not finite!!!"<<std::endl;
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


void geometricCtrl::armingCallback(const ros::TimerEvent& event){
  nh_.param<std::string>("/runAlg", runAlg, "lawnMower");
    
  if (newDataFlag && runAlg.compare("info")==0 && g_geodetic_converter.isInitialised() && !std::any_of(newPosData.begin(),newPosData.end(), [](bool v) {return !v;}) && !std::any_of(newVelData.begin(),newVelData.end(), [](bool v) {return !v;}) && newRefData || tuneAtt || tuneRate){
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
    else{ // i am simulating
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
    //here
  }
}
  
void geometricCtrl::cmdloopCallback(const ros::TimerEvent& event){    
  
  //double t2=clock();
  if (runAlg.compare("info")==0 && newDataFlag || tuneAtt || tuneRate){
    if (!tuneAtt && !tuneRate){    
      if (quadMode.data==1){
	holdPos_ = mavPos_;
	holdPos_(2)=0.0;
	
	if((current_state_.armed || current_state_.mode.compare("OFFBOARD")==0) && runAlg.compare("info")==0){
	  quadMode.data=2;
	}
      }
      else if(quadMode.data==2){      
	Eigen::Vector3d errorPos_, errorVel_, errorPos_noCA, errorVel_filtered;
	Eigen::Matrix3d R_ref;
	if(current_state_.mode.compare("OFFBOARD")==0 && current_state_.armed){
	  holdPos_(2)+=0.04;
	  if (holdPos_(2)>1){
	    holdPos_(2)=1;
	  }
	}
	
	errorPos_   = mavPos_ - holdPos_;
	errorVel_   = mavVel_;    
	errorSum_  += errorPos_;    
	
	action_int_ = (Kint_.asDiagonal()*errorSum_);
	if (action_int_.norm()>max_fb_acc_){
	  action_int_=(max_fb_acc_/ action_int_.norm())*action_int_;
	}
	
	/// Compute BodyRate commands using differential flatness
	/// Controller based on Faessler 2017
	a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_ + action_int_; //feedforward term for trajectory error
	// if(a_fb(2) < -max_fb_acc_) a_fb(2) = -max_fb_acc_;
	if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;    
	
	a_des = a_fb - g_;

	Eigen::Vector3d controlA_p = Kpos_.asDiagonal() * errorPos_;
	Eigen::Vector3d controlA_v = Kvel_.asDiagonal() * errorVel_;
	Eigen::Vector3d controlA_i = action_int_;

	geometry_msgs::TwistStamped controlAction, controlActionInt;
	controlAction.header.stamp = ros::Time::now();
	controlActionInt.header.stamp = ros::Time::now();
	controlAction.twist.linear.x = controlA_p(0);
	controlAction.twist.linear.y = controlA_p(1);
	controlAction.twist.linear.z = controlA_p(2);
	controlAction.twist.angular.x = controlA_v(0);
	controlAction.twist.angular.y = controlA_v(1);
	controlAction.twist.angular.z = controlA_v(2);

	controlActionInt.twist.linear.x = controlA_i(0);
	controlActionInt.twist.linear.y = controlA_i(1);
	controlActionInt.twist.linear.z = controlA_i(2);
	
	controlActionPub_.publish(controlAction);
	controlActionIntPub_.publish(controlActionInt);


	//a_des_history[ades_idx]=a_des;    
	
	//a_des_filtered = std::accumulate(a_des_history.begin(), a_des_history.end(), Eigen::Vector3d(0,0,0)) / a_des_history.size();

	//ades_idx++;

	//if(ades_idx>a_des_history.size()) ades_idx=0;
    
    
	q_des = acc2quaternion(a_des, mavYaw_);
	cmdBodyRate_ = attcontroller(q_des, a_des, mavAtt_); //Calculate BodyRate
	cmdBodyRate_(2)=0;


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
	
	if (((targetPos_noCA-mavPos_).norm() < 0.1 && targetPos_noCA(2)>0.75 && mavVel_.norm()<.35) && mavPos_(2)>0.75 && holdPos_(2)==1.0 && std::all_of(newPosData.begin(),newPosData.end(), [](bool v) {return v;}) && std::all_of(newVelData.begin(),newVelData.end(), [](bool v) {return v;})){
	  if(target_trajectoryID_==0){
	    quadMode.data=2; //stay in hold mode
	  }
	  else{
	    quadMode.data=3;
	  }
	}
	else{
	  ROS_INFO_THROTTLE(.5,"quadMode= %d, waiting until quad is still: targetPos_noCA-mavPos_=%f, mavVel=%f, mavPos_(2)=%f, newPos=%d, newVel=%d", quadMode.data, (targetPos_noCA-mavPos_).norm(), mavVel_.norm(), mavPos_(2), std::all_of(newPosData.begin(),newPosData.end(), [](bool v) {return v;}), std::all_of(newVelData.begin(),newVelData.end(), [](bool v) {return v;}));
	}
      }
      else if(quadMode.data==3){
	computeBodyRateCmd(false);
      }
    }
    else if (tuneAtt || tuneRate){
      computeBodyRateCmd(false);
    }
    
    pubReferencePose();
    pubRateCommands();
  }
  
  quadModePub_.publish(quadMode);

  //t2=((float)clock()-t2)/CLOCKS_PER_SEC;
  //ROS_INFO_THROTTLE(1,"QM: %d cmd loop: %f", quadMode, t2);
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
  current_state_ = *msg;
  if (current_state_.mode.compare("OFFBOARD")!=0 || !current_state_.armed){
    quadMode.data=1;
    std::fill(newVelData.begin(), newVelData.end(), false);
    std::fill(newPosData.begin(), newPosData.end(), false);
    std::fill(ctrs.begin(), ctrs.end(), 0);
  }  
}

void geometricCtrl::statusloopCallback(const ros::TimerEvent& event){  
}

void geometricCtrl::pubReferencePose(){
  referencePoseMsg_.header.stamp = ros::Time::now();
  referencePoseMsg_.header.frame_id = "world";
  referencePoseMsg_.pose.position.x = targetPos_noCA(0);
  referencePoseMsg_.pose.position.y = targetPos_noCA(1);
  referencePoseMsg_.pose.position.z = targetPos_noCA(2);

  referencePoseMsg_.pose.orientation.w = q_des(0);
  referencePoseMsg_.pose.orientation.x = q_des(1);
  referencePoseMsg_.pose.orientation.y = q_des(2);
  referencePoseMsg_.pose.orientation.z = q_des(3);
  referencePosePub_.publish(referencePoseMsg_);

  referencePoseMsgCA_=referencePoseMsg_;
  if (quadMode.data<3){
    referencePoseMsgCA_.pose.position.x = holdPos_(0);
    referencePoseMsgCA_.pose.position.y = holdPos_(1);
    referencePoseMsgCA_.pose.position.z = holdPos_(2);
  }
  else{
    referencePoseMsgCA_.pose.position.x = targetPos_(0);
    referencePoseMsgCA_.pose.position.y = targetPos_(1);
    referencePoseMsgCA_.pose.position.z = targetPos_(2);
  }
  referencePosePubCA_.publish(referencePoseMsgCA_);
}

void geometricCtrl::pubRateCommands(){
  angularVelMsg_.header.stamp = ros::Time::now();
  angularVelMsg_.header.frame_id= "world";
  angularVelMsg_.body_rate.x = cmdBodyRate_(0);
  angularVelMsg_.body_rate.y = -1*cmdBodyRate_(1);
  angularVelMsg_.body_rate.z = -1*cmdBodyRate_(2);
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
    nh_.getParam("geometric_controller/attctrl_tau_p", attctrl_tau_p);
    nh_.getParam("geometric_controller/attctrl_tau_i", attctrl_tau_i);
    nh_.getParam("geometric_controller/attctrl_tau_d", attctrl_tau_d);

    nh_.param<double>("geometric_controller/max_tau_i", max_tau_i, 0.5);
    nh_.param<double>("geometric_controller/norm_thrust_const_", norm_thrust_const_, 0.1);
    nh_.param<double>("geometric_controller/max_fb_acc_", max_fb_acc_, 5.0);
    nh_.param<double>("geometric_controller/max_rollRate", max_rollRate, 10);
    nh_.param<double>("geometric_controller/max_pitchRate", max_pitchRate, 10);
    nh_.param<double>("geometric_controller/max_yawRate", max_yawRate, 10);

    
    ROS_INFO_THROTTLE(3,"attctrl_tau_p=[%f, %f, %f], attctrl_tau_i=[%f, %f, %f], attctrl_tau_d=[%f, %f, %f] desiredAtt: [%f, %f, %f]", attctrl_tau_p[0], attctrl_tau_p[1], attctrl_tau_p[2], attctrl_tau_i[0], attctrl_tau_i[1], attctrl_tau_i[2], attctrl_tau_d[0], attctrl_tau_d[1], attctrl_tau_d[2],desiredAtt[0], desiredAtt[1], desiredAtt[2]);
    
    a_ref(0) = desiredAtt[0];
    a_ref(1) = desiredAtt[1];
    a_ref(2) = desiredAtt[2];

    // chosen such that thrust is near hovering
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
    errorSum_  += errorPos_;    
        
    action_int_ = (Kint_.asDiagonal()*errorSum_);
    if (action_int_.norm()>max_fb_acc_){
      action_int_=(max_fb_acc_/ action_int_.norm())*action_int_;
    }

    // errorVel_history[ev_idx]=errorVel_;

    // errorVel_filtered = std::accumulate(errorVel_history.begin(), errorVel_history.end(), Eigen::Vector3d(0,0,0)) / errorVel_history.size();
    
        
    a_ref = targetAcc_;

    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    q_ref = acc2quaternion(a_ref - g_, mavYaw_);
    //R_ref = quat2RotMatrix(q_ref);
    a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_ + action_int_; //feedforward term for trajectory error


    Eigen::Vector3d controlA_p = Kpos_.asDiagonal() * errorPos_;
    Eigen::Vector3d controlA_v = Kvel_.asDiagonal() * errorVel_;
    Eigen::Vector3d controlA_i = action_int_;    
      

    geometry_msgs::TwistStamped controlAction, controlActionInt;
    controlAction.header.stamp = ros::Time::now();
    controlActionInt.header.stamp = ros::Time::now();
    controlAction.twist.linear.x = controlA_p(0);
    controlAction.twist.linear.y = controlA_p(1);
    controlAction.twist.linear.z = controlA_p(2);
    controlAction.twist.angular.x = controlA_v(0);
    controlAction.twist.angular.y = controlA_v(1);
    controlAction.twist.angular.z = controlA_v(2);

    controlActionInt.twist.linear.x = controlA_i(0);
    controlActionInt.twist.linear.y = controlA_i(1);
    controlActionInt.twist.linear.z = controlA_i(2);
	
    controlActionPub_.publish(controlAction);
    controlActionIntPub_.publish(controlActionInt);

    
    
    if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;    
    
    //a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * targetVel_; //Rotor drag
    // a_des = a_fb + a_ref - a_rd - g_;
    a_des = a_fb + a_ref - g_;


    if (Eigen::Vector2d(a_des(0),a_des(1)).norm()>xyAccelMax){
      Eigen::Vector2d new_a_des = xyAccelMax/Eigen::Vector2d(a_des(0),a_des(1)).norm() * Eigen::Vector2d(a_des(0),a_des(1));
      a_des(0) = new_a_des(0);
      a_des(1) = new_a_des(1);
    }

    // if(a_des(0) > xyAccelMax ) a_des(0) = xyAccelMax;
    // if(a_des(0) < -xyAccelMax) a_des(0) = -xyAccelMax;
    // if(a_des(1) > xyAccelMax ) a_des(1) = xyAccelMax;
    // if(a_des(1) < -xyAccelMax) a_des(1) = -xyAccelMax;
        
    //a_des_history[ades_idx]=a_des;    

    //a_des_filtered = std::accumulate(a_des_history.begin(), a_des_history.end(), Eigen::Vector3d(0,0,0)) / a_des_history.size();

    //ades_idx++;

    //if(ades_idx>a_des_history.size()) ades_idx=0;

    // std::normal_distribution<double> Gsampler(0.0,.1);
    // std::mt19937 generator;
    // unsigned seed_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    // generator.seed(seed_time);

    // a_des(0)=a_des(0)+Gsampler(generator);
    // a_des(1)=a_des(1)+Gsampler(generator);
    // a_des(2)=a_des(2)+Gsampler(generator);

    
    
    q_des = acc2quaternion(a_des, mavYaw_);



    // q_des(0)=q_des(0)+Gsampler(generator);
    // q_des(1)=q_des(1)+Gsampler(generator);
    // q_des(2)=q_des(2)+Gsampler(generator);
    // q_des(3)=q_des(3)+Gsampler(generator);
    //q_des.norm();

      
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
  // Eigen::Vector4d ratecmd;
  // Eigen::Vector4d qe, q_inv, inverse;
  // Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb;  
  q_inv = inverse.asDiagonal() * curr_att;
  qe = quatMultiplication(q_inv, ref_att);

  geometry_msgs::QuaternionStamped errorQ;
  errorQ.header.stamp= ros::Time::now();
  errorQ.quaternion.w = qe(0);
  errorQ.quaternion.x = qe(1);
  errorQ.quaternion.y = qe(2);
  errorQ.quaternion.y = qe(3);  
  error_attPub_.publish(errorQ);  
  
  signQe = std::copysign(1.0, qe(0));
  //signQe_dot = std::copysign(1.0, qe_dot(0));
  
  actionAtt << signQe*qe(1), signQe*qe(2), signQe*qe(3);
  //actionDotAtt<< signQe_dot*qe_dot(1), signQe_dot*qe_dot(2), signQe_dot*qe_dot(3);  
  sumAtt += actionAtt;
  
  // prevent windup
  if (sumAtt(0)>max_tau_i){
    sumAtt(0)=max_tau_i;
  }
  else if(sumAtt(0)<-max_tau_i){
    sumAtt(0)=-max_tau_i;
  }
  
  if (sumAtt(1)>max_tau_i){
    sumAtt(1)=max_tau_i;
  }
  else if(sumAtt(1)<-max_tau_i){
    sumAtt(1)=-max_tau_i;
  }
  
  if (sumAtt(2)>max_tau_i){
    sumAtt(2)=max_tau_i;
  }
  else if(sumAtt(2)<-max_tau_i){
    sumAtt(2)=-max_tau_i;
  }

  // remove uneccesarry calculations
  rollRate  = 2.0/attctrl_tau_p[0] * actionAtt(0) + 2.0/attctrl_tau_i[0]*sumAtt(0);// + 2.0/attctrl_tau_d[0] * mavRate_(0);
  pitchRate = 2.0/attctrl_tau_p[1] * actionAtt(1) + 2.0/attctrl_tau_i[1]*sumAtt(1);// + 2.0/attctrl_tau_d[1] * mavRate_(1);
  yawRate   = 2.0/attctrl_tau_p[2] * actionAtt(2) + 2.0/attctrl_tau_i[2]*sumAtt(2);//+ 2.0/attctrl_tau_d[2] * mavRate_(2);

  if (rollRate>max_rollRate){
    ratecmd(0) = max_rollRate;
  }
  else if (rollRate<(-max_rollRate)){
    ratecmd(0) = -max_rollRate;
  }
  else{
    ratecmd(0)=rollRate;
  }

  if (pitchRate>max_pitchRate){
    ratecmd(1) = max_pitchRate;
  }
  else if (pitchRate<(-max_pitchRate)){
    ratecmd(1) = -max_pitchRate;
  }
  else{
    ratecmd(1)=pitchRate;
  }

  if (yawRate>max_yawRate){
    ratecmd(2) = max_yawRate;
  }
  else if (yawRate<(-max_yawRate)){
    ratecmd(2) = -max_yawRate;
  }
  else{
    ratecmd(2)=yawRate;
  }

  
  rotmat = quat2RotMatrix(curr_att);
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
