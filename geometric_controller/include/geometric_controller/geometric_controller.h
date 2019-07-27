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
#include <time.h>

#include <Eigen/Dense>
#include <mav_msgs/Actuators.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include "enif_iuc/AgentSource.h"
#include "mps_driver/MPS.h"
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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>

#include <RVO.h>
#include <dlib/matrix.h>
#include <Eigen/Eigen>
#define STATS_USE_EIGEN
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "geodetic_utils/geodetic_conv.hpp"

#include "kalman.hpp"

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
    ros::Subscriber agentState_sub;

    ros::Subscriber keybrdSub_;
    ros::Subscriber mavstateSub_;
    ros::Subscriber mavposeSub_, gzmavposeSub_, local_sub;
    ros::Subscriber mavtwistSub_;
    ros::Subscriber rcSub_, source_sub;  
    ros::Publisher rotorVelPub_, angularVelPub_;
    ros::Publisher referencePosePub_, referencePosePubCA_, referenceCA_setpointPub_;
    ros::Publisher des_attRefPub_, error_attPub_;
    ros::Publisher cur_attRefPub_;
    ros::Publisher mavPosVelPub_;
    ros::Publisher mavAccelPub_, quadModePub_;
    ros::Publisher bPub_, hzPub_;
    ros::Publisher obstaclesPub_;
    ros::Publisher controlActionPub_, controlActionIntPub_;
    std::vector<ros::Publisher> agentPos_pub, agentVel_pub;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient frame_client;
    ros::ServiceServer ctrltriggerServ_;
    ros::Timer cmdloop_timer_, statusloop_timer_, arming_timer_, checkData_timer_;
    ros::Time last_request_, reference_request_now_, reference_request_last_;

    int mode, tpvh, target_trajectoryID_;
    int AGENT_NUMBER;
    std::vector<double> desiredRate, desiredAtt, attctrl_tau_p, attctrl_tau_d, attctrl_tau_i;
    std::vector<ros::Time> agentInfo_time;
    std::vector<int> ctrs;
    std::vector<std::chrono::time_point<std::chrono::system_clock>> startTimes;
    std::vector<std::chrono::duration<double>> e_time;

    bool tuneRate, tuneAtt, avoiding, timeFlag, obstaclesOn, tunePosVel, newSourceData, outerBox;
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
    double kp_rot_, kd_rot_, gainCA;
    double reference_request_dt_;
    double xs, ys, zs, sourceObjSize; // source position for obstacle
    /* double attctrl_tau_; */
    //Eigen::Vector3d attctrl_tau_;
    double norm_thrust_const_;
    double max_fb_acc_, max_fb_jerk_, max_rollRate, max_pitchRate, max_yawRate, max_rollPitch, takeOffThrust, max_tau_i;
    float radius, timeH;
    mavros_msgs::SetMavFrame mav_frame;
    std::vector<bool> newPosData, newVelData;
    bool newRefData, avoidAgents, newDataFlag, simSetup;
    int numAgents;
    std_msgs::Int16 quadMode;
    geometry_msgs::TwistStamped b_msg;

    Eigen::Vector4d ratecmd;
    Eigen::Vector4d qe, q_inv, q_inv_dot, inverse, qe_prev, qe_dot;
    Eigen::Matrix3d rotmat;

    Eigen::Vector3d sumAtt, actionAtt, cmdRate, actionDotAtt;
    double rollRate, pitchRate, yawRate;
    int signQe, signQe_dot;

    std::vector<KalmanFilter> kfs;
    Eigen::MatrixXd A; // System dynamics matrix
    Eigen::MatrixXd C; // Output matrix
    Eigen::MatrixXd Q; // Process noise covariance
    Eigen::MatrixXd R; // Measurement noise covariance
    Eigen::MatrixXd P; // Measurement noise covariance
    int n, m; // number of states, number of measurmentes
    double kl_dt_x;
    bool useKalman;
    std::vector<bool> kfInit;

    int transCtr;
    std::chrono::time_point<std::chrono::system_clock> start;


    std::vector<Eigen::Vector3d> errorVel_history;
    /* std::vector<Eigen::Vector3d> errorVel_history; */
    int ev_idx;
    std::string runAlg;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::AttitudeTarget angularVelMsg_;
    geometry_msgs::PoseStamped referencePoseMsg_, referencePoseMsgCA_;
    geometry_msgs::Vector3Stamped des_eulerRefMsg_;
    geometry_msgs::Vector3Stamped cur_eulerRefMsg_;
    geometry_msgs::AccelStamped accel_CA;
    geometry_msgs::TwistStamped referenceSetpointCAMsg_;

    std::vector<Eigen::Vector3d> targetPos_history, targetVel_history;
    int t_idx;
    
    
    Eigen::Vector3d goalPos_, targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_, targetCA_vel, targetCA_pos, targetPos_noCA, targetVel_noCA, targetAcc_noCA, targetPos_noCA_prev_, targetVel_noCA_prev_;
    Eigen::Vector3d mavPos_, mavVel_, mavRate_, holdPos_;
    double mavYaw_, xyAccelMax, holdYaw_;
    Eigen::Vector3d a_des, a_fb,a_fb_prev, a_ref, a_rd, g_, a_des_filtered, action_int_;
    std::vector<Eigen::Vector3d> a_des_history;
    int ades_idx;
        
    Eigen::Vector4d mavAtt_, q_ref, q_des;
    Eigen::Vector4d cmdBodyRate_; //{wx, wy, wz, Thrust}
    Eigen::Vector3d Kpos_, Kvel_, D_, Kpos_noCA_, Kint_;
    Eigen::Vector3d errorSum_;

    RVO::Vector2 errorsumOri_;
    RVO::Vector2 desVel;
    int mavState;

    RVO::RVOSimulator* sim;
    visualization_msgs::Marker obstacleMsg;

    void pubMotorCommands();
    void pubRateCommands();
    void pubReferencePose();
    void localCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void sourceCallback(const enif_iuc::AgentSource &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& odomMsg);
    void rc_command_callback(const mavros_msgs::RCIn::ConstPtr &new_message);
    void targetCallback(const geometry_msgs::TwistStamped& msg);
    void targetAccelCallback(const geometry_msgs::AccelStamped& msg);
    void flattargetCallback(const controller_msgs::FlatTarget& msg);
    void keyboardCallback(const geometry_msgs::Twist& msg);
    void agentStateCallback(const std_msgs::UInt8& msg);
    void cmdloopCallback(const ros::TimerEvent& event);
    void armingCallback(const ros::TimerEvent& event);
    void checkDataCallback(const ros::TimerEvent& event);    

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
    dlib::matrix<double> SS;      // state space of the target states.
    double v_max;    
    std::vector<RVO::Vector2> goals;
    geometricCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void computeBodyRateCmd(bool ctrl_mode);
    Eigen::Vector4d quatMultiplication(Eigen::Vector4d &q, Eigen::Vector4d &p);
    Eigen::Vector4d attcontroller(Eigen::Vector4d &ref_att, Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    virtual ~ geometricCtrl();
};


#endif
