#pragma once

#include <Eigen/Core>

#include "geometry/quat.h"


template <int N>
class WaypointTrajectory
{

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

enum : int {
  wpN = 0,
  wpE = 1,
  wpD = 2,
  wpYaw = 3,
  wpZ = 4,
};

typedef Eigen::Matrix<double, xZ, 1> StateVector;
typedef Eigen::Matrix<double, dxZ, 1> ErrStateVector;
typedef Eigen::Matrix<double, uZ, 1> InputVector;

public:
  WaypointTrajectory(const Eigen::Matrix<double, 4, N> &waypoints,
                     double waypoint_duration, double hover_throttle)
  {
    waypoints_ = waypoints;
    waypoint_duration_ = waypoint_duration;
    hover_throttle_ = hover_throttle;
  }

  void increaseWaypointIndex()
  {
    waypoint_index_++;

    if (waypoint_index_ >= N)
      waypoint_index_ = 0;
  }

  void getCommandedState(const double &t, StateVector &x_c, InputVector &u_r)
  {
    if ((t - last_waypoint_time_) > waypoint_duration_)
    {
      this->increaseWaypointIndex();
      last_waypoint_time_ = t;
    }

    Eigen::Vector4d current_waypoint =
        waypoints_.template block<4, 1>(0, waypoint_index_);

    x_c.setZero();
    x_c(xPOS+0, 0) = current_waypoint(wpN);
    x_c(xPOS+1, 0) = current_waypoint(wpE);
    x_c(xPOS+2, 0) = current_waypoint(wpD);
    
    const double roll = 0.;
    const double pitch = 0.;
    const double yaw = current_waypoint(wpYaw);
    quat::Quatd q_c(roll, pitch, yaw);

    x_c.template block<4, 1>(xATT, 0) = q_c.elements();

    u_r.setZero();
    u_r(uTHROTTLE) = hover_throttle_;
  }

private:
  Eigen::Matrix<double, 4, N> waypoints_;

  double waypoint_duration_;
  double last_waypoint_time_ = 0.;
  int waypoint_index_ = 0;

  double hover_throttle_;
};

