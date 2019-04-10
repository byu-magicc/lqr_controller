#ifndef LQR_H
#define LQR_H

#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <lqr_controller/Figure8Config.h>

#include <memory>
#include <Eigen/Core>

#include "lin_alg_tools/care.h"
#include "geometry/quat.h"

#include "lqr/figure8.h"
#include "lqr/waypoints.h"
#include "lqr/logger.h"

enum : int {
  xPOS = 0,
  xATT = 3,
  xVEL = 7,
  xOMEGA = 10,
  xZ = 13
};

enum : int {
  dxPOS = 0,
  dxVEL = 3,
  dxATT = 6,
  dxZ = 9
};

enum : int {
  uTHROTTLE = 0,
  uOMEGA = 1,
  uZ = 4,
};

typedef Eigen::Matrix<double, xZ, 1> StateVector;
typedef Eigen::Matrix<double, dxZ, 1> ErrStateVector;
typedef Eigen::Matrix<double, uZ, 1> InputVector;
typedef Eigen::Matrix<double, dxZ, dxZ> ErrStateErrStateMatrix;
typedef Eigen::Matrix<double, dxZ, uZ> ErrStateInputMatrix;
typedef Eigen::Matrix<double, uZ, dxZ> InputErrStateMatrix;
typedef Eigen::Matrix<double, uZ, uZ> InputInputMatrix;

namespace lqr
{
class LQRController
{
public:
  LQRController();
  void computeControl(const StateVector &x, const StateVector &x_c,
                      const InputVector &ur, InputVector &u);

private:
  const double grav_val_ = 9.8;
  double hover_throttle_;
  double drag_const_;

  double max_pos_err_;
  double max_alt_err_;
  double max_ang_err_;
  double max_vel_err_;
  double max_throttle_err_;
  double max_omega_err_;

  double max_throttle_c_;
  double min_throttle_c_;
  double max_omega_c_;
  double min_omega_c_;

  double start_time_ = 0.;
  double current_time_ = 0.;

  ErrStateErrStateMatrix A_;
  ErrStateInputMatrix B_;
  ErrStateErrStateMatrix Q_;
  InputInputMatrix R_;

  ErrStateErrStateMatrix P_;
  InputErrStateMatrix K_;
  CareSolver<dxZ, uZ> care_solver;

  StateVector x_;
  StateVector x_c_;
  ErrStateVector delta_x_;

  InputVector u_;
  InputVector ur_;

  void saturateInput(InputVector &u);
  void saturateErrorVec(Eigen::Vector3d &err, double max_err);
  void saturateErrorVec(Eigen::Vector3d &err, double max_err, double max_err2);

  // Trajectory Stuff
  bool use_fig8_ = false;
  std::unique_ptr<Figure8> fig8_traj_ = nullptr;
  bool use_waypoints_ = false;
  std::unique_ptr<WaypointTrajectory<5>> wp_traj_ = nullptr;

  // Logger
  std::unique_ptr<Logger> log_ = nullptr;

  // ROS stuff
  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Publisher command_pub_;

  rosflight_msgs::Command command_msg_;

  void publishCommand(const InputVector &u);
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);

  dynamic_reconfigure::Server<lqr_controller::Figure8Config> server_;
  dynamic_reconfigure::Server<lqr_controller::Figure8Config>::CallbackType func_;
  void reconfigureCallback(lqr_controller::Figure8Config &config,
                            uint32_t level);
};
}

#endif /* LQR_H */
