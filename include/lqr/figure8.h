#pragma once

#include <Eigen/Core>

#include "geometry/quat.h"


class Figure8
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

typedef Eigen::Matrix<double, xZ, 1> StateVector;
typedef Eigen::Matrix<double, dxZ, 1> ErrStateVector;
typedef Eigen::Matrix<double, uZ, 1> InputVector;

public:
  Figure8(double x_width, double y_width, double z_width, double period,
          StateVector &_x0, double _hover_throttle)
  {
    ax_ = x_width;
    ay_ = y_width;
    az_ = z_width;
    T_ = period;
    x0_ = _x0;
    hover_throttle_ = _hover_throttle;

    x0_p_ = x0_.block<3, 1>(xPOS, 0);
  }

  Eigen::Vector3d getAccel(const double &t)
  {
    static const double f = T_ / (2.0 * M_PI);
    Eigen::Vector3d a_I;
    a_I.x() = -ax_ * f * f * std::sin(t * f);
    a_I.y() = -ay_ * 4.0 * f * f * std::sin(2.0 * t * f);
    a_I.z() = -az_ * f * f * std::sin(t * f);
    static const Eigen::Vector3d grav_vec(0., 0., grav_);
    a_I = a_I + grav_vec;
    return a_I;
  }

  Eigen::Vector3d getPosition(const double &t)
  {
    static const double f = T_ / (2.0 * M_PI);
    Eigen::Vector3d p_I;
    p_I.x() = x0_p_.x() + ax_ * std::sin(t * f);
    p_I.y() = x0_p_.y() + ay_ * std::sin(2.0 * t * f);
    p_I.z() = x0_p_.z() + az_ * std::sin(t * f);
    return p_I;
  }

  Eigen::Vector3d getVelocity(const double &t)
  {
    static const double f = T_ / (2.0 * M_PI);
    Eigen::Vector3d v_I;
    v_I.x() = ax_ * f * std::cos(t * f);
    v_I.y() = ay_ * 2.0 * f * std::cos(2.0 * t * f);
    v_I.z() = az_ * f * std::cos(t * f);
    return v_I;
  }

  void getCommandedState(const double &t, StateVector &x_c, InputVector &u_r)
  {
    static const Eigen::Vector3d e_z(0., 0., 1.);

    Eigen::Vector3d a_I = getAccel(t);
    quat::Quatd x_c_q =
        quat::Quatd::from_two_unit_vectors(a_I.normalized(), e_z);
    
    x_c.block<3, 1>(xPOS, 0) = getPosition(t);
    x_c.block<3, 1>(xVEL, 0) = x_c_q.rotp(getVelocity(t));
    x_c.block<4, 1>(xATT, 0) = x_c_q.elements();

    const double dt = 1e-2;
    Eigen::Vector3d a_I_prev = getAccel(t - dt);
    Eigen::Vector3d a_I_next = getAccel(t + dt);

    quat::Quatd qprev =
        quat::Quatd::from_two_unit_vectors(a_I_prev.normalized(), e_z);
    quat::Quatd qnext =
        quat::Quatd::from_two_unit_vectors(a_I_next.normalized(), e_z);

    u_r(0) = a_I.norm() / grav_ * hover_throttle_;
    u_r.bottomRows<3>() = (qnext - qprev) / (2.0 * dt);

    x_c.block<3, 1>(xOMEGA, 0) = (qnext - qprev) / (2.0 * dt);
  }

private:
  double ax_, ay_, az_, T_;
  double hover_throttle_;
  const double grav_ = 9.8;
  StateVector x0_;
  Eigen::Vector3d x0_p_;
};

