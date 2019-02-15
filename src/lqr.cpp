#include "lqr/lqr.h"

#include "geometry/quat.h"
#include "geometry/support.h"

namespace lqr
{
LQRController::LQRController() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{
  nh_private_.getParam("hover_throttle", hover_throttle_);
  nh_private_.getParam("drag_constant", drag_const_);
  nh_private_.getParam("lqr_max_pos_error", max_pos_err_);
  nh_private_.getParam("lqr_max_vel_error", max_vel_err_);
  nh_private_.getParam("lqr_max_ang_error", max_ang_err_);
  nh_private_.getParam("lqr_max_throttle_error", max_throttle_err_);
  nh_private_.getParam("lqr_max_omega_error", max_omega_err_);
  nh_private_.getParam("lqr_max_throttle_command", max_throttle_c_);
  nh_private_.getParam("lqr_min_throttle_command", min_throttle_c_);
  nh_private_.getParam("lqr_max_omega_command", max_omega_c_);
  nh_private_.getParam("lqr_min_omega_command", min_omega_c_);

  Q_.setZero();
  Q_.block<3, 3>(dxPOS, dxPOS) =
      (1. / max_pos_err_) * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(dxVEL, dxVEL) =
      (1. / max_vel_err_) * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(dxATT, dxATT) =
      (1. / max_ang_err_) * Eigen::Matrix3d::Identity();

  R_.setZero();
  R_(uTHROTTLE, uTHROTTLE) = (1. / max_throttle_err_);
  R_.block<3, 3>(uOMEGA, uOMEGA) =
      (1. / max_omega_err_) * Eigen::Matrix3d::Identity();

  std::cout << "LQR INIT:" << std::endl;
  std::cout << "LQR Q: " << std::endl << Q_ << std::endl;
  std::cout << "LQR R: " << std::endl << R_ << std::endl;

  // Set up Publishers and Subscriber
  state_sub_ =
      nh_.subscribe("estimate", 1, &LQRController::stateCallback, this);
  imu_sub_ =
      nh_.subscribe("imu_data", 1, &LQRController::imuCallback, this);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
}

void LQRController::computeControl(const StateVector &x, const StateVector &x_c,
                                   const InputVector &ur,
                                   InputVector &u)
{
  // x - current state
  // x_c - desired state
  // u - output [F (throttle 0 -> 1.0), omega_x, omega_y, omega_z].T
  // ur - reference output

  Eigen::Matrix3d M = drag_const_ * Eigen::Matrix3d::Identity();
  M(2, 2) = 0.;
  static const Eigen::Vector3d e3(0., 0., 1.);
  static const Eigen::Vector3d grav_vec(0., 0., grav_val_);

  const Eigen::Vector3d vel_b = x.block<3, 1>(xVEL, 0);
  const quat::Quatd q_I_b(x.block<4, 1>(xATT, 0));
  const Eigen::Matrix3d R_I_b = q_I_b.R();
  const Eigen::Vector3d omega_b = omega_current_;
  const Eigen::Matrix3d skew_omega = skew(omega_b);
  const Eigen::Matrix3d skew_vel = skew(vel_b);

  A_.setZero();
  A_.block<3, 3>(dxPOS, dxVEL) = R_I_b.transpose();
  A_.block<3, 3>(dxPOS, dxATT) = -R_I_b.transpose() * skew_vel;

  A_.block<3, 3>(dxVEL, dxVEL) = -M - skew_omega;
  A_.block<3, 3>(dxVEL, dxATT) = skew(R_I_b * grav_vec);

  A_.block<3, 3>(dxATT, dxATT) = -skew_omega;

  B_.setZero();
  B_.block<3, 1>(dxVEL, uTHROTTLE) = -(grav_val_ / hover_throttle_) * e3;
  B_.block<3, 3>(dxVEL, uOMEGA) = skew_vel;

  B_.block<3, 3>(dxATT, uOMEGA) = Eigen::Matrix3d::Identity();

  P_.setZero();
  care_solver.solve(P_, A_, B_, Q_, R_);
  K_ = -R_.inverse() * B_.transpose() * P_;

  Eigen::Vector3d pos_err = x.block<3, 1>(xPOS, 0) - x_c.block<3, 1>(xPOS, 0);
  saturateErrorVec(pos_err, max_pos_err_);

  Eigen::Vector3d vel_err = x.block<3, 1>(xVEL, 0) - x_c.block<3, 1>(xVEL, 0);
  saturateErrorVec(vel_err, max_vel_err_);

  const quat::Quatd q_I_b_c(x_c.block<4, 1>(xATT, 0));
  Eigen::Vector3d q_err = q_I_b - q_I_b_c; // boxminus
  saturateErrorVec(q_err, max_ang_err_);

  delta_x_.block<3, 1>(dxPOS, 0) = pos_err;
  delta_x_.block<3, 1>(dxVEL, 0) = vel_err;
  delta_x_.block<3, 1>(dxATT, 0) = q_err;

  u.setZero();
  u = ur + K_ * delta_x_;

  saturateInput(u);
}

void LQRController::saturateInput(InputVector &u)
{
  if (u(0) > max_throttle_c_)
    u(0) = max_throttle_c_;
  else if (u(0) < min_throttle_c_)
    u(0) = min_throttle_c_;

  for (int i = 1; i < 4; i++)
  {
    if (u(i) > max_omega_c_)
      u(i) = max_omega_c_;
    else if (u(i) < min_omega_c_)
      u(i) = min_omega_c_;
  }
}

void LQRController::saturateErrorVec(Eigen::Vector3d& err, double max_err)
{
  for (int i = 0; i < 3; i++)
  {
    if (std::abs(err(i)) > max_err)
      err(i) = err(i) / std::abs(err(i)) * max_err;
  }
}

void LQRController::publishCommand(const InputVector &u)
{
  command_msg_.header.stamp = ros::Time::now();
  command_msg_.mode =
      rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  command_msg_.F = u(uTHROTTLE);
  command_msg_.x = u(uOMEGA+0);
  command_msg_.y = u(uOMEGA+1);
  command_msg_.z = u(uOMEGA+2);

  command_pub_.publish(command_msg_);
}

void LQRController::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if(start_time_ == 0)
  {
    start_time_ = msg->header.stamp.toSec();
    return;
  }

  current_time_ = msg->header.stamp.toSec() - start_time_;

  x_(xPOS+0) = msg->pose.pose.position.x;
  x_(xPOS+1) = msg->pose.pose.position.y;
  x_(xPOS+2) = msg->pose.pose.position.z;
  
  x_(xVEL+0) = msg->twist.twist.linear.x;
  x_(xVEL+1) = msg->twist.twist.linear.y;
  x_(xVEL+2) = msg->twist.twist.linear.z;

  x_(xATT+0) = msg->pose.pose.orientation.w;
  x_(xATT+1) = msg->pose.pose.orientation.x;
  x_(xATT+2) = msg->pose.pose.orientation.y;
  x_(xATT+3) = msg->pose.pose.orientation.z;


  // TODO set commanded state
  x_c_.setZero();
  x_c_(xPOS+0) = 5.;
  x_c_(xPOS+1) = -5.;
  x_c_(xPOS+2) = -5.;
  x_c_(xATT) = 1.;

  // TODO set reference input
  ur_.setZero();
  ur_(uTHROTTLE) = hover_throttle_;

  this->computeControl(x_, x_c_, ur_, u_);
  this->publishCommand(u_);
}

void LQRController::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  omega_current_(0) = msg->angular_velocity.x;
  omega_current_(1) = msg->angular_velocity.y;
  omega_current_(2) = msg->angular_velocity.z;
}
}
