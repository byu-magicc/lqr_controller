#include <ros/ros.h>
#include "lqr/lqr.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lqr_controller");

  lqr::LQRController lqr_controller;

  ros::spin();

  return 0;
}
