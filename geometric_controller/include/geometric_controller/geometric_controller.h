//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
//#include <kindr/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <mav_msgs/Actuators.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <controller_msgs/FlatTarget.h>
#include <std_srvs/SetBool.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/SetMavFrame.h>

#include <rvo2/RVO.h>
#include <dlib/matrix.h>
#include <Eigen/Eigen>
#define STATS_USE_EIGEN
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "geodetic_utils/geodetic_conv.hpp"

#define MODE_ROTORTHRUST  1
#define MODE_BODYRATE     2
#define MODE_BODYTORQUE   3

using namespace std;
using namespace Eigen;
class geometricCtrl
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber odomSub_;
    ros::Subscriber referenceSub_;
    ros::Subscriber accelReferenceSub_;
    ros::Subscriber flatreferenceSub_;
    ros::Subscriber agentSub_;

    ros::Subscriber keybrdSub_;
    ros::Subscriber mavstateSub_;
    ros::Subscriber mavposeSub_, gzmavposeSub_;
    ros::Subscriber mavtwistSub_;
    ros::Subscriber rcSub_;  
    ros::Publisher rotorVelPub_, angularVelPub_;
    ros::Publisher referencePosePub_;
    ros::Publisher des_eulerRefPub_;
    ros::Publisher cur_eulerRefPub_;
    ros::Publisher mavPosVelPub_;
    ros::Publisher mavAccelPub_;
    ros::Publisher bPub_;
    ros::Publisher obstaclesPub_;
    std::vector<ros::Publisher> agentPos_pub, agentVel_pub;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient frame_client;
    ros::ServiceServer ctrltriggerServ_;
    ros::Timer cmdloop_timer_, statusloop_timer_;
    ros::Time last_request_, reference_request_now_, reference_request_last_;

    int mode, tpvh;
    int AGENT_NUMBER;
    std::vector<double> desiredRate, desiredAtt;
    std::vector<ros::Time> agentInfo_time;
    bool tuneRate, tuneAtt, avoiding, timeFlag, obstaclesOn;
    ros::Time finishedAvoid_time, avoid_time;
    mavros_msgs::RCIn  RCin;
    int num_rotors_;
    string mav_name_;
    std::string agentName;   
    int max_motor_speed_;
    bool fail_detec_, ctrl_enable_;
    int ctrl_mode_;
    bool landing_commanded_;
    bool use_gzstates_, sim_enable_;
    double kp_rot_, kd_rot_;
    double reference_request_dt_;
    /* double attctrl_tau_; */
    Eigen::Vector3d attctrl_tau_;
    double norm_thrust_const_;
    double max_fb_acc_;
    float radius;
    mavros_msgs::SetMavFrame mav_frame;
    std::vector<bool> newPosData, newVelData;
    bool newRefData, avoidAgents, newDataFlag;
    int numAgents;
    geometry_msgs::TwistStamped b_msg;

    std::vector<Eigen::Vector3d> errorVel_history;
    /* std::vector<Eigen::Vector3d> errorVel_history; */
    int ev_idx;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::AttitudeTarget angularVelMsg_;
    geometry_msgs::PoseStamped referencePoseMsg_;
    geometry_msgs::Vector3Stamped des_eulerRefMsg_;
    geometry_msgs::Vector3Stamped cur_eulerRefMsg_;
    geometry_msgs::AccelStamped accel_CA;

    std::vector<Eigen::Vector3d> targetPos_history, targetVel_history;
    
    Eigen::Vector3d goalPos_, targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_, targetCA_vel, targetCA_pos, targetPos_noCA, targetVel_noCA, targetAcc_noCA, targetPos_noCA_prev_, targetVel_noCA_prev_;
    Eigen::Vector3d mavPos_, mavVel_, mavRate_;
    double mavYaw_;
    Eigen::Vector3d a_des, a_fb, a_ref, a_rd, g_, a_des_filtered;
    std::vector<Eigen::Vector3d> a_des_history;
    int ades_idx;
    
    Eigen::Vector4d mavAtt_, q_ref, q_des;
    Eigen::Vector4d cmdBodyRate_; //{wx, wy, wz, Thrust}
    Eigen::Vector3d Kpos_, Kvel_, D_, Kpos_noCA_;

    RVO::RVOSimulator* sim;

    void pubMotorCommands();
    void pubRateCommands();
    void pubReferencePose();
    void odomCallback(const nav_msgs::OdometryConstPtr& odomMsg);
    void rc_command_callback(const mavros_msgs::RCIn::ConstPtr &new_message);
    void targetCallback(const geometry_msgs::TwistStamped& msg);
    void targetAccelCallback(const geometry_msgs::AccelStamped& msg);
    void flattargetCallback(const controller_msgs::FlatTarget& msg);
    void keyboardCallback(const geometry_msgs::Twist& msg);
    void cmdloopCallback(const ros::TimerEvent& event);
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
    void mavposeCallback(const geometry_msgs::PoseStamped& msg);
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg);
    void gzmavposeCallback(const gazebo_msgs::ModelStates& msg);
    void statusloopCallback(const ros::TimerEvent& event);
    bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void agentsCallback(const enif_iuc::AgentMPS &msg);
    void wait4Home(void);
    void setupScenario(void);
    void updateAgents(void);
    void updateGoal(void);
    void updateCA_velpos(void);
    double b_sigMoid(double x, double c, double a);
      
    Eigen::Vector4d acc2quaternion(Eigen::Vector3d vector_acc, double yaw);
    Eigen::Vector4d rot2Quaternion(Eigen::Matrix3d R);
    Eigen::Matrix3d quat2RotMatrix(Eigen::Vector4d q);
    geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);

  public:
    geodetic_converter::GeodeticConverter g_geodetic_converter;
    double H_latitude,H_longitude,H_altitude;
    dlib::matrix<double> xt, vt;
    double v_max;    
    std::vector<RVO::Vector2> goals;
    geometricCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void computeBodyRateCmd(bool ctrl_mode);
    Eigen::Vector4d quatMultiplication(Eigen::Vector4d &q, Eigen::Vector4d &p);
    Eigen::Vector4d attcontroller(Eigen::Vector4d &ref_att, Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    virtual ~ geometricCtrl();
};


#endif
