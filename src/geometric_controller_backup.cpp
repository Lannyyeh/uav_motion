//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "uav_motion/geometric_controller.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh),
                                                                                             nh_private_(nh_private),
                                                                                             fail_detec_(false),
                                                                                             ctrl_enable_(true),
                                                                                             landing_commanded_(false),
                                                                                             feedthrough_enable_(false),
                                                                                             node_state(WAITING_FOR_HOME_POSE),
                                                                                             init_x_(0.0),
                                                                                             init_y_(0.0),
                                                                                             init_z_(1.0)

{

  referenceSub_ = nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ = nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("/command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this, ros::TransportHints().tcpNoDelay());
  mavstateSub_ = nh_.subscribe("/mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this, ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());
  mavThrustSub_ = nh_.subscribe("/mavros/getThrustSetpoint/thrustSetpoint", 1, &geometricCtrl::mavThrustCallback, this, ros::TransportHints().tcpNoDelay());
  mavimuSub_ = nh_.subscribe("/mavros/imu/data", 1, &geometricCtrl::mavimuCallback, this, ros::TransportHints().tcpNoDelay());

  ctrltriggerServ_ = nh_.advertiseService("tigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback, this);    // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback, this); // Define timer for constant loop rate

  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("/geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);

  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05); // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);   // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_x", init_x_, init_x_);
  nh_private_.param<double>("init_y", init_y_, init_y_);
  nh_private_.param<double>("init_z", init_z_, init_z_);

  targetPos_ << init_x_, init_y_, init_z_; // Initial Position
  ROS_INFO("target position: %.2f, %.2f, %.2f", targetPos_(0), targetPos_(1), targetPos_(2));
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

  D_ << dx_, dy_, dz_;

  tau << tau_x, tau_y, tau_z;

  double position_p_x, position_p_y, position_p_z;
  double position_Kp1_x, position_Kp1_y, position_Kp1_z;
  double position_Kp2_x, position_Kp2_y, position_Kp2_z;

  nh_.param<double>("/Controller/position_p/x", position_p_x, 3.0);
  nh_.param<double>("/Controller/position_p/y", position_p_y, 3.0);
  nh_.param<double>("/Controller/position_p/z", position_p_z, 3.0);
  nh_.param<double>("/Controller/position_Kp1/x", position_Kp1_x, 1.2);
  nh_.param<double>("/Controller/position_Kp1/y", position_Kp1_y, 1.2);
  nh_.param<double>("/Controller/position_Kp1/z", position_Kp1_z, 2.0);
  nh_.param<double>("/Controller/position_Kp2/x", position_Kp2_x, 0.01);
  nh_.param<double>("/Controller/position_Kp2/y", position_Kp2_y, 0.01);
  nh_.param<double>("/Controller/position_Kp2/z", position_Kp2_z, 0.02);
  nh_.param<double>("/Controller/position_sigma", sigma_1, 0.02);
  
  nh_.param<double>("/Controller/gamma_1", gamma_1, 0.02);
  nh_.param<double>("/uav/mass", vehicle_mass, 1.0);
  
  Kp_1 << position_Kp1_x, position_Kp1_y, position_Kp1_z;
  Kp_2 << position_Kp2_x, position_Kp2_y, position_Kp2_z;
  lambda_1 << position_p_x / position_Kp1_x, position_p_y / position_Kp1_y, position_p_z / position_Kp1_z;
  mass_hat = vehicle_mass;

  outFile.open("/home/ubuntu/uav_motion_ws/record/record.csv");
  outFileParamCheck.open("/home/ubuntu/uav_motion_ws/record/ParamCheck.csv");
  outFileInnerCheck.open("/home/ubuntu/uav_motion_ws/record/InnerCheck.csv");
}

geometricCtrl::~geometricCtrl()
{
  // Destructor
}

void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg)
{

  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg)
{

  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);

  if (msg.type_mask == 1)
  {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = Eigen::Vector3d::Zero();
  }
  else if (msg.type_mask == 2)
  {

    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  }
  else if (msg.type_mask == 4)
  {

    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  }
  else
  {

    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = toEigen(msg.snap);
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg)
{
  if (!velocity_yaw_)
  {
    double yaw = double(msg.data);
    // while (yaw > 3.141592653589793) yaw = yaw - 6.283185307179586;
    // while (yaw < -3.14159265358979) yaw = yaw + 6.28318530717958;

    mavYaw_ = yaw;
  }
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg)
{

  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!received_home_pose)
  {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position);
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg)
{
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}

void geometricCtrl::mavimuCallback(const sensor_msgs::Imu &msg)
{
  mavAcc_ = toEigen(msg.linear_acceleration);
}


void geometricCtrl::mavThrustCallback(const mavros_msgs::ThrustSPFromPX4 &msg)
{
  thrustFromPX4 = msg.xyz[2];
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  node_state = LANDING;
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event)
{
  switch (node_state)
  {
  case WAITING_FOR_HOME_POSE:
    waitForPredicate(&received_home_pose, "Waiting for home pose...");
    ROS_INFO("Got pose! Drone Ready to be armed.");
    node_state = MISSION_EXECUTION;
    break;
  case MISSION_EXECUTION:
    if (!feedthrough_enable_)
      computeBodyRateCmd(cmdBodyRate_);
    computeThrustCmd_asmc();
    pubReferencePose(targetPos_, q_des);
    pubRateCommands(cmdBodyRate_);
    appendPoseHistory();
    pubPoseHistory();
    break;
  case LANDING:
  {
    geometry_msgs::PoseStamped landingmsg;
    landingmsg.header.stamp = ros::Time::now();
    landingmsg.pose = home_pose_;
    landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
    target_pose_pub_.publish(landingmsg);
    node_state = LANDED;
    ros::spinOnce();
    break;
  }
  case LANDED:
    ROS_INFO("Landed. Please set to position control and disarm.");
    cmdloop_timer_.stop();
    break;
  }
  if (current_state_.mode == "OFFBOARD")
  {
    outFileParamCheck << "Kp1" << "," << Kp_1(0) << "," << Kp_1(1) << "," << Kp_1(2) << ",\n";
    outFileParamCheck << "Kp2" << "," << Kp_2(0) << "," << Kp_2(1) << "," << Kp_2(2) << ",\n";
    outFileParamCheck << "lambda1" << "," << lambda_1(0) << "," << lambda_1(1) << "," << lambda_1(2) << ",\n";
    outFileParamCheck << "Kpos_" << "," << Kpos_(0) << "," << Kpos_(1) << "," << Kpos_(2) << ",\n";
    outFileParamCheck << "Kvel_" << "," << Kvel_(0) << "," << Kvel_(1) << "," << Kvel_(2) << ",\n";
    
    outFile << "setpoint" << "," << targetPos_(0) << "," << targetPos_(1) << "," << targetPos_(2) << "," << "\n";
    outFile << "currentPose" << "," << mavPos_(0) << "," << mavPos_(1) << "," << mavPos_(2) << "," << "\n";
    outFile << "targetVel" << "," << targetVel_(0) << "," << targetVel_(1) << "," << targetVel_(2) << "," << "\n";
    outFile << "currentVel" << "," << mavVel_(0) << "," << mavVel_(1) << "," << mavVel_(2) << "," << "\n";
    outFile << "currentAcc" << "," << mavAcc_(0) << "," << mavAcc_(1) << "," << mavAcc_(2) << "," << "\n";
    outFile << "acc_des" << "," << a_des(0) << "," << a_des(1) << "," << a_des(2) << "," << "\n";
    outFile << "acc_feedback" << "," << a_fb(0) << "," << a_fb(1) << "," << a_fb(2) << "," << "\n";
    outFile << "cmdBodyRate" << "," << cmdBodyRate_(0) << "," << cmdBodyRate_(1) << "," << cmdBodyRate_(2) << "," << cmdBodyRate_(3) << "," << "\n";
    // outFile << "thrustCheck" << "," << thrustFromPX4 << "," << "\n";
    outFile << "acc_des_asmc" << "," << acc_des_asmc(0) << "," << acc_des_asmc(1) << "," << acc_des_asmc(2) << "," << "\n";
    outFile << "acc_fb_asmc" << "," << acc_fb_asmc(0) << "," << acc_fb_asmc(1) << "," << acc_fb_asmc(2) << "," << "\n";
    outFile << "mass" << "," << vehicle_mass << "," << mass_hat << "," << thrust_adaptive << "," << "\n";

    outFileInnerCheck << "term1" << "," << term1(0) << "," << term1(1) << "," << term1(2) << "," << "\n";
    outFileInnerCheck << "term2" << "," << term2(0) << "," << term2(1) << "," << term2(2) << "," << "\n";
    outFileInnerCheck << "mass_diff" << "," << mass_diff << "," << "\n";
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  current_state_ = *msg;
}

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event)
{
  if (sim_enable_)
  {
    // Enable OFFBoard mode and arm automatically
    // This is only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
    {
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    }
    else
    {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
      {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude)
{
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd)
{
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  // msg.body_rate.x = 0.0;
  // msg.body_rate.y = 0.0;
  // msg.body_rate.z = 0.0;
  msg.type_mask = 128; // Ignore orientation messages
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

void geometricCtrl::pubPoseHistory()
{
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;

  posehistoryPub_.publish(msg);
}

void geometricCtrl::pubSystemStatus()
{
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196; // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void geometricCtrl::appendPoseHistory()
{
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_)
  {
    posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation)
{
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd)
{

  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = targetAcc_;
  if (velocity_yaw_)
  {
    mavYaw_ = std::atan2(-1.0 * mavVel_(1), mavVel_(0));
  }

  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - targetPos_;
  const Eigen::Vector3d vel_error = mavVel_ - targetVel_;

  a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; // feedforward term for trajectory error
  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; // Clip acceleration if reference is too large

  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * targetVel_; // Rotor drag
  a_des = a_fb + a_ref - a_rd - g_;

  // q_des = acc2quaternion(a_des, mavYaw_);

  // if (ctrl_mode_ == ERROR_GEOMETRIC)
  // {
  //   bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_); // Calculate BodyRate
  // }
  // else
  // {
  //   bodyrate_cmd = attcontroller(q_des, a_des, mavAtt_); // Calculate BodyRate
  // }
}

void geometricCtrl::computeThrustCmd_asmc()
{
  // compute thrust and torque commands using adaptive sliding mode controller
  // controller based on Xu 2024
  const Eigen::Vector3d e_1 = mavPos_ - targetPos_;
  const Eigen::Vector3d e_1_diff = mavVel_ - targetVel_;

  Eigen::Vector3d s_1 = e_1_diff + lambda_1.asDiagonal() * e_1;
  Eigen::Vector3d delta_1 = s_1;
  // if (sigma_1 != 0)
  //   delta_1 = s_1 - sigma_1 * sat(s_1 / sigma_1);

  Eigen::Matrix<double, 3, 1> p_r_diff2 = targetAcc_ - lambda_1.asDiagonal() * e_1_diff;
  // Calculate desired force
  // the gravity direction is opposite to that in our paper
  Eigen::Vector3d force_tmp;
  force_tmp = vehicle_mass * (-g_ + p_r_diff2)- Kp_1.asDiagonal() * delta_1;
  // if (sigma_1 != 0)
  //   force_tmp = vehicle_mass * (-g_ + p_r_diff2) - Kp_1.asDiagonal() * delta_1 - Kp_2.asDiagonal() * sat(s_1 / sigma_1);

  acc_des_asmc = force_tmp / vehicle_mass;
  acc_fb_asmc = acc_des_asmc - targetAcc_ - (-g_);

  // Clip acceleration if the part of feedback is too large
  if (acc_fb_asmc.norm() > max_fb_acc_)
  {
    acc_fb_asmc = (max_fb_acc_ / acc_fb_asmc.norm()) * acc_fb_asmc;
  }
  acc_des_asmc = acc_fb_asmc + targetAcc_ + (-g_);

  q_des = acc2quaternion(acc_des_asmc, mavYaw_);

  if (ctrl_mode_ == ERROR_GEOMETRIC)
  {
    cmdBodyRate_ = geometric_attcontroller(q_des, acc_des_asmc, mavAtt_); // Calculate BodyRate
  }
  else
  {
    cmdBodyRate_ = attcontroller(q_des, acc_des_asmc, mavAtt_); // Calculate BodyRate
  }

  if (current_state_.mode == "OFFBOARD")
  {
    double dt = 0.01;
    mass_diff = - gamma_1 * delta_1.transpose() * (-g_ + p_r_diff2);
    mass_diff = mass_diff * dt;
    mass_hat = mass_hat + mass_diff;
    term1 = delta_1;
    term2 = -g_ + p_r_diff2;
  }
}

Eigen::Vector4d geometricCtrl::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p)
{
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
      p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
      p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d geometricCtrl::quat2RotMatrix(const Eigen::Vector4d q)
{
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

Eigen::Vector4d geometricCtrl::rot2Quaternion(const Eigen::Matrix3d R)
{
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0)
  {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  }
  else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
  {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  }
  else if (R(1, 1) > R(2, 2))
  {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  }
  else
  {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d vector_acc, double yaw)
{
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0),
      xb_des(1), yb_des(1), zb_des(1),
      xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

Eigen::Vector4d geometricCtrl::attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att)
{
  // Geometric attitude controller
  // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter attitude control: Technical report. ETH Zurich, 2013.

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
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); // Calculate thrust

  return ratecmd;
}

Eigen::Vector4d geometricCtrl::geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att)
{
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
  // The original paper inputs moment commands, but for offboard control angular rate commands are sent

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat;   // Rotation matrix of current atttitude
  Eigen::Matrix3d rotmat_d; // Rotation matrix of desired attitude
  Eigen::Vector3d zb;
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);
  ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);

  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); // Calculate thrust
  double scaling_factor = mass_hat / vehicle_mass; 
  ratecmd(3) = ratecmd(3) * scaling_factor;

  return ratecmd;
}

Eigen::Matrix3d geometricCtrl::matrix_hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d m;
  // Sanity checks on M
  m << 0.0, -v(2), v(1),
      v(2), 0.0, -v(0),
      -v(1), v(0), 0.0;
  return m;
}

Eigen::Vector3d geometricCtrl::matrix_hat_inv(const Eigen::Matrix3d &m)
{
  Eigen::Vector3d v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

void geometricCtrl::getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel)
{
  pos = mavPos_;
  att = mavAtt_;
  vel = mavVel_;
  angvel = mavRate_;
}

void geometricCtrl::getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel)
{
  pos = mavPos_ - targetPos_;
  vel = mavVel_ - targetVel_;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req,
                                        std_srvs::SetBool::Response &res)
{
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
}

void geometricCtrl::setBodyRateCommand(Eigen::Vector4d bodyrate_command)
{
  cmdBodyRate_ = bodyrate_command;
}

void geometricCtrl::setFeedthrough(bool feed_through)
{
  feedthrough_enable_ = feed_through;
}

void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config, uint32_t level)
{

  if (max_fb_acc_ != config.max_acc)
  {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  }
  else if (Kpos_x_ != config.Kp_x)
  {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  }
  else if (Kpos_y_ != config.Kp_y)
  {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  }
  else if (Kpos_z_ != config.Kp_z)
  {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  }
  else if (Kvel_x_ != config.Kv_x)
  {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  }
  else if (Kvel_y_ != config.Kv_y)
  {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  }
  else if (Kvel_z_ != config.Kv_z)
  {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_z_, -Kvel_z_;
}

// using namespace RAI;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  geometricCtrl *geometricController = new geometricCtrl(nh, nh_private);

  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig> srv;
  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig>::CallbackType f;
  f = boost::bind(&geometricCtrl::dynamicReconfigureCallback, geometricController, _1, _2);
  srv.setCallback(f);

  ros::spin();
  return 0;
}
