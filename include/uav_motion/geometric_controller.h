//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>

#include <Eigen/Dense>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/ThrustSPFromPX4.h>

#include <uav_motion/release.h>
// #include "uav_motion/generatePath.h"

#include <controller_msgs/FlatTarget.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <dynamic_reconfigure/server.h>
#include <geometric_controller/GeometricControllerConfig.h>

#include "uav_motion/disturbance_estimator.hpp"
#include "uav_motion/sliding_window_filter.hpp"

#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2

using namespace std;
using namespace Eigen;

enum class MAV_STATE
{
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class geometricCtrl
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber referenceSub_;
  ros::Subscriber flatreferenceSub_;
  ros::Subscriber multiDOFJointSub_;
  
  ros::Subscriber mavimuSub_;
  ros::Subscriber mavstateSub_;
  ros::Subscriber mavposeSub_, gzmavposeSub_;
  ros::Subscriber mavtwistSub_;
  ros::Subscriber yawreferenceSub_;
  ros::Subscriber mavThrustSub_;
  ros::Subscriber mavBatterySub_;

  ros::Subscriber waypointsStatusSub_;  
  
  ros::Publisher rotorVelPub_, angularVelPub_, target_pose_pub_;
  ros::Publisher referencePosePub_;
  ros::Publisher posehistoryPub_;
  ros::Publisher systemstatusPub_;
  ros::Publisher jointAnglePub_;

  ros::Publisher flightStatePub_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceServer ctrltriggerServ_;
  // ros::ServiceServer land_service_;
  ros::ServiceServer release_service_;
  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::Time last_request_, reference_request_now_, reference_request_last_;

  ofstream outFile;
  ofstream outFileParamCheck;
  ofstream outFileInnerCheck;

  string mav_name_;
  bool fail_detec_, ctrl_enable_, feedthrough_enable_;
  int ctrl_mode_;
  bool landing_commanded_;
  bool sim_enable_;
  bool velocity_yaw_;
  double kp_rot_, kd_rot_;
  double reference_request_dt_;
  double attctrl_tau_;
  double norm_thrust_const_, norm_thrust_offset_;
  double max_fb_acc_;
  double dx_, dy_, dz_;
  double init_x_, init_y_, init_z_;

  mavros_msgs::State current_state_;
  mavros_msgs::SetMode offb_set_mode_;
  mavros_msgs::CommandBool arm_cmd_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;
  bool control_available_;  // offboard && armed && not killed

  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
  Eigen::Vector3d mavPos_, mavVel_, mavAcc_, mavRate_;
  double mavYaw_;
  double thrustFromPX4;
  Eigen::Vector3d g_;
  Eigen::Vector4d mavAtt_, q_des;
  Eigen::Vector4d cmdBodyRate_; //{wx, wy, wz, Thrust}
  Eigen::Vector4d cmdIdleRate_; //{wx, wy, wz, Thrust}
  Eigen::Vector4d cmdActual_;
  Eigen::Vector3d Kpos_, Kvel_, D_;
  Eigen::Vector3d a0, a1, tau;
  double tau_x, tau_y, tau_z;
  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
  Eigen::Vector3d a_fb, a_des;
  int posehistory_window_;

  // for asmc controller
  Eigen::Vector3d lambda_1, Kp_1, Kp_2;
  double sigma_1, gamma_1;
  double vehicle_mass, mass_hat, mass_diff;
  Eigen::Vector3d acc_des_asmc, acc_fb_asmc;
  Eigen::Vector3d term1, term2;
  double thrust_adaptive;
  double batteryVoltage;

  // for impedance controller
  Eigen::Vector3d force_feedback, force_3d;
  Eigen::Vector3d Kp;

  // for external force estimator
  int estimator_enable;
  DisturbEstimator force_ext_estimator;
  SlidingWindowFilter force_ext_filter;
  Eigen::Vector3d K_f;
  Eigen::Vector3d force_ext, force_ext_filtered;

  // for releasing process
  int mission_, test_mode;
  bool call_for_release;
  bool dropping_detected;
  double vehicle_idle_thrust, vehicle_release_thrust, vehicle_release_rate_z;
  Eigen::Vector4d cmdReleasingRate_; //{wx, wy, wz, Thrust}
  Eigen::Vector3d droppingPos_;

  // for docking
  bool ready_to_dock_;
  Eigen::Vector3d dockingPos_;
  double docking_final_altitude, docking_actual_altitude;
  bool use_impedance_control;
  double force_desired;
  Eigen::Vector3d force_desired_vec;
  int docking_count, retry_count;

  // some constrains
  double max_rate_yaw;

  // for morphing
  int control_step;
  int morph;
  double joint_max_angle, joint_min_angle;
  double morphing_start_time, morphing_time;
  double joint_angle_;

  void pubMotorCommands();
  void pubRateCommands(const Eigen::Vector4d &cmd);
  void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
  void pubPoseHistory();
  void pubSystemStatus();
  void pubFlightState();

  void pubJointAngle();

  void appendPoseHistory();
  
  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void targetCallback(const geometry_msgs::TwistStamped &msg);
  void flattargetCallback(const controller_msgs::FlatTarget &msg);
  void yawtargetCallback(const std_msgs::Float32 &msg);
  void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
  
  void keyboardCallback(const geometry_msgs::Twist &msg);
  void cmdloopCallback(const ros::TimerEvent &event);
  
  void mavimuCallback(const sensor_msgs::Imu &msg);
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void mavThrustCallback(const mavros_msgs::ThrustSPFromPX4 &msg);
  void mavBatteryCallback(const sensor_msgs::BatteryState &msg);

  bool releaseCallback(uav_motion::release::Request &req, uav_motion::release::Response &res);
  void waypointsStatusCallback(const std_msgs::Bool &msg);
  void checkDockingPos();

  void statusloopCallback(const ros::TimerEvent &event);
  bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  // bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d q);
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);
  
  void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd);
  void computeThrustCmd_asmc();
  void computeThrustCmd_normal();
  Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
  Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
  Eigen::Vector4d geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);


  inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p)
  {
    Eigen::Vector3d ev3(p.x, p.y, p.z);
    return ev3;
  }

  inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3)
  {
    Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
    return ev3;
  }

  enum FlightState
  {
    WAITING_FOR_HOME_POSE,
    WAITING_FOR_OFFBOARD_TRIGGER,
    // WAITING_TO_RELEASE,
    REALEASING,
    // INTERMEDIATE_STATE,
    TRACKING,
    DOCKING,
    FINISHED
  } node_state;

  enum DockingSubState {
    INIT_ALIGN,
    ASCEND_TRACK,
    IMPEDANCE_CTRL,
    RETRY_ALIGN,
    IDLE
  } docking_state; 

  template <class T>
  void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0)
  {
    ros::Rate pause(hz);
    ROS_INFO_STREAM(msg);
    while (ros::ok() && !(*pred))
    {
      ros::spinOnce();
      pause.sleep();
    }
  };
  geometry_msgs::Pose home_pose_;
  bool received_home_pose;


public:
  void dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config, uint32_t level);
  geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  void getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel);
  void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel);
  void setBodyRateCommand(Eigen::Vector4d bodyrate_command);
  void setFeedthrough(bool feed_through);
  virtual ~geometricCtrl();

  static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d vector_acc, double yaw);
  static Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d R);
  static Eigen::Matrix3d matrix_hat(const Eigen::Vector3d &v);
  static Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m);
};

#endif
