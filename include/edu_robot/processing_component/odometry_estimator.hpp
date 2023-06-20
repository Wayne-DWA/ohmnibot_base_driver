/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/processing_component/processing_component.hpp"

#include <edu_robot/angle.hpp>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eigen3/Eigen/Core>

namespace eduart {
namespace robot {
namespace processing {

class OdometryEstimator : public ProcessingComponent
{
public:
  struct Parameter {

  };

  OdometryEstimator(const Parameter parameter);
  ~OdometryEstimator() override = default;

  void process(const Eigen::Vector3f& measured_velocity);
  nav_msgs::Odometry getOdometryMessage(const std::string& robot_base_frame, const std::string& odom_frame) const;
  geometry_msgs::TransformStamped getTfMessage(const std::string& robot_base_frame, const std::string& odom_frame) const;

private:
  AnglePiToPi _orientation = 0.0;
  float _position_x = 0.0f;
  float _position_y = 0.0f;
  float _linear_velocity_x = 0.0f;
  float _linear_velocity_y = 0.0f;
  float _angular_velocity_z = 0.0f;
};

} // end namespace processing
} // end namespace robot
} // end namespace eduart
