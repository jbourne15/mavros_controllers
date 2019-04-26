//  May/2018, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#define TRAJ_STATIONARY 0
#define TRAJ_CIRCLE 1
#define TRAJ_LAMNISCATE 2
#define TRAJ_POLY 3

#define MODE_PRIMITIVES 1
#define MODE_REFERENCE 2

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation_ros/feasibility_base.h>
#include <mav_trajectory_generation_ros/input_constraints.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <mavros_msgs/State.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>

#include "trajectory_publisher/trajectory.h"

using namespace std;
using namespace Eigen;
class trajectoryPublisher
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber local_sub, state_sub;
  ros::Publisher trajectoryPub_;
  ros::Publisher referencePub_;
  ros::Publisher markers_pub;
  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_;

  nav_msgs::Path refTrajectory_;
  geometry_msgs::TwistStamped refState_;

  int counter;
  int mode_;
  Eigen::Vector3d target_initpos;
  Eigen::Vector3d traj_axis_;
  Eigen::Vector3d p_targ, v_targ;
  Eigen::Vector3d mavPos_;
  Eigen::Vector4d mavAtt_;
  std::string agentName;
  double traj_radius_, traj_omega_;
  double theta_ = 0.0;
  double controlUpdate_dt_;
  double trigger_time_;
  double init_pos_x_, init_pos_y_, init_pos_z_;
  int target_trajectoryID_;
  mavros_msgs::State state;

  trajectory motionPrimitives_;

  /* mav_trajectory_generation::Vertex::Vector vertices; */
  int dimension;// = 3;
  int derivative_to_optimize;// = mav_trajectory_generation::derivative_order::SNAP;
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  mav_trajectory_generation::Segment::Vector segments;
  mav_trajectory_generation::Trajectory trajectory_poly;
  visualization_msgs::MarkerArray markers;

public:  
  trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void setTrajectory(int ID);
  void setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius, Eigen::Vector3d initpos);
  void setTrajectoryTheta(double in);
  double getTrajectoryOmega();
  double getTrajectoryUpdateRate();
  void moveReference();
  void pubrefTrajectory();
  void pubrefState();
  void stateCallback(const mavros_msgs::State &msg);
  void localCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
  Eigen::Vector3d getTargetPosition();
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_message);
  void getPolyTrajectory(void);

  };


#endif
