#include <ohmnibot_base_driver/ohmnibot_base_driver.h>
// #include "edu_robot/color.hpp"
#include "edu_robot/hardware_error.hpp"
// #include "edu_robot/lighting.hpp"

#include "edu_robot/mode.hpp"
#include "ohmnibot_base_driver/Mode.h"
// #include "edu_robot/msg/detail/mode_struct.hpp"
// #include "edu_robot/msg_conversion.hpp"
#include "edu_robot/processing_component/odometry_estimator.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>

#include "ros/ros.h"

#include <memory>
#include <cstdio>
#include <exception>
#include <functional>
#include <stdexcept>
#include <cstddef>

namespace eduart {
namespace robot {
namespace ohmnibot {

using namespace std::chrono_literals;

Ohmnibot::Ohmnibot(std::string robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface)
    :_logger_prefix("ohmnibot_base_driver: ")
    , _hardware_interface(std::move(hardware_interface))
    {
        if(!loadParameter()) ROS_ERROR_STREAM(_logger_prefix <<"failed to load parameters from server");
        _pub_odometry = _nh.advertise<nav_msgs::Odometry>(_parameter.pub_odom_topic, 1);
        _sub_twist = _nh.subscribe<geometry_msgs::Twist>(_parameter.sub_twist_topic, 1,&Ohmnibot::cbCmdVel, this);
        // _pub_status_report = _nh.advertise<edu_robot::msg::RobotStatusReport>("status_report", 1);
        // _pub_kinematic_description = _nh.advertise<edu_robot::msg::RobotKinematicDescription>("ohmnibot_kinematic_description", 1);
        // _srv_set_mode = _nh.advertiseService<edu_robot::srv::SetMod>("set_mode",std::bind(&Ohmnibot::callbackServiceSetMode, this, std::placeholders::_1, std::placeholders::_2));

        _odometry_component = std::make_shared<processing::OdometryEstimator>(processing::OdometryEstimator::Parameter{});
        // if(_parameter.enable_joint_state_publisher)
        // {
        //     int n_joints = 0;            
        //     _pub_joint_state = _nh.advertise<sensor_msgs::JointState>(_parameter.topic_joint_states, 1);
        //     _joint_state_msg.name.push_back("joint_wheel_front_left");
        //     _joint_state_msg.name.push_back("joint_wheel_front_right");
        //     _joint_state_msg.name.push_back("joint_wheel_back_right");
        //     _joint_state_msg.name.push_back("joint_wheel_back_left");
        //     n_joints += 4;
        //     _joint_state_msg.position.resize(n_joints);
        //     _joint_state_msg.velocity.resize(n_joints);
        // }
      _hardware_interface->enable();

    }
bool Ohmnibot::loadParameter()
{
    ROS_INFO_STREAM(_logger_prefix <<"load parameters from server");
    ros::NodeHandle privateNh("~");

    privateNh.param("tf_base_frame", _parameter.tf_base_frame, std::string("base_link"));
    privateNh.param("tf_footprint_frame", _parameter.tf_footprint_frame, std::string("base_footprint"));
    privateNh.param("publish_tf_odom", _parameter.publish_tf_odom, true);
    privateNh.param("sub_twist_topic", _parameter.sub_twist_topic, std::string("cmd_vel"));
    privateNh.param("pub_odom_topic", _parameter.pub_odom_topic, std::string("odometry"));
    privateNh.param("enable_joint_state_publisher", _parameter.enable_joint_state_publisher, false);
    privateNh.param("topic_pub_joint_states", _parameter.topic_joint_states, std::string("base_joint_states"));
    privateNh.param("timeout_cmd_vel", _parameter.timeout_cmd_vel,0.5);

    privateNh.param("mecanum.length.x", _parameter.mecanum.length.x,0.25);
    privateNh.param("mecanum.length.y", _parameter.mecanum.length.y,0.36);
    privateNh.param("mecanum.wheel_diameter", _parameter.mecanum.wheel_diameter,0.1);
    return true;
}
void Ohmnibot::cbCmdVel(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  // Kick Watch Dog
  // _timer_status_report->reset();
  _stamp_cmd_vel     = ros::Time::now();
  try {
    // \todo maybe a size check would be great!
    Eigen::Vector3f velocity_cmd(twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);

    // Calculate wheel rotation speed using provided kinematic matrix.
    // Apply velocity reduction if a limit is reached.
    Eigen::VectorXf radps = _kinematic_matrix * velocity_cmd;
    float reduce_factor = 1.0f;
  
    for (Eigen::Index i = 0; i < radps.size(); ++i) {
      reduce_factor = std::min(
        std::abs(_motor_controllers[i]->parameter().max_rpm / Rpm::fromRadps(radps(i))), reduce_factor
      );
    }
    if(ros::Time::now().toSec() > (_stamp_cmd_vel.toSec() + _parameter.timeout_cmd_vel))
    {
        //if timeout, the robot should not move.
        reduce_factor = 0.0;
    }
    for (Eigen::Index i = 0; i < radps.size(); ++i) {
        _motor_controllers[i]->setRpm(Rpm::fromRadps(radps(i)) * reduce_factor);
    }
    // Calculating Odometry and Publishing it
    Eigen::VectorXf radps_measured(_motor_controllers.size());

    for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
      radps_measured(i) = _motor_controllers[i]->getMeasuredRpm().radps();
    }

    const Eigen::Vector3f velocity_measured = _inverse_kinematic_matrix * radps_measured;
    _odometry_component->process(velocity_measured);
    _pub_odometry.publish(_odometry_component->getOdometryMessage(
      getFrameIdPrefix() + _parameter.tf_footprint_frame, getFrameIdPrefix() + "odom"
    ));

    if (_parameter.publish_tf_odom) {
      _tf_broadcaster.sendTransform(_odometry_component->getTfMessage(
        getFrameIdPrefix() + _parameter.tf_footprint_frame, getFrameIdPrefix() + "odom"
      ));
    }
  }
  catch (HardwareError& ex) {
    ROS_ERROR_STREAM(_logger_prefix <<"Hardware error occurred while trying to set new values for motor controller."
                                      << " what() = " << ex.what());                                      
  }
  catch (std::exception& ex) {
    ROS_ERROR_STREAM(_logger_prefix <<"Error occurred while trying to set new values for motor controller."
                                      << " what() = " << ex.what());     
  }
}

void Ohmnibot::registerMotorController(std::shared_ptr<MotorController> motor_controller)
{
  const auto search = _motor_controllers.find(motor_controller->id());

  if (search != _motor_controllers.end()) {
    ROS_ERROR_STREAM(_logger_prefix <<"Motor controller \"" << motor_controller->name() << "\" already contained"
                                      << " in lighting register. Can't add it twice.");
    return;      
  }

  _motor_controllers[motor_controller->id()] = motor_controller;
}


std::string Ohmnibot::getFrameIdPrefix() const
{
  // remove slash at the beginning
  // std::string frame_id_prefix(get_effective_namespace().begin() + 1, get_effective_namespace().end());
  std::string frame_id_prefix;
  // add slash at the end if it is missing
  if (frame_id_prefix.back() != '/') {
    frame_id_prefix.push_back('/');
  }
  return frame_id_prefix;
}

void Ohmnibot::switchKinematic(const Mode mode)
{
  _kinematic_matrix = getKinematicMatrix(mode);
  _inverse_kinematic_matrix = _kinematic_matrix.completeOrthogonalDecomposition().pseudoInverse();

  // edu_robot::msg::RobotKinematicDescription msg;

  // msg.k.cols = _kinematic_matrix.cols();
  // msg.k.rows = _kinematic_matrix.rows();
  // msg.k.data.resize(_kinematic_matrix.cols() * _kinematic_matrix.rows());

  // for (Eigen::Index row = 0; row < _kinematic_matrix.rows(); ++row) {
  //   for (Eigen::Index col = 0; col < _kinematic_matrix.cols(); ++col) {
  //     msg.k.data[row * _kinematic_matrix.cols() + col] = _kinematic_matrix(row, col);
  //   }
  // }
  // for (const auto& motor : _motor_controllers) {
  //   msg.wheel_limits.push_back(motor.second->parameter().max_rpm);
  // }

  // _pub_kinematic_description->publish(msg);
}

void Ohmnibot::initialize(eduart::robot::HardwareComponentFactory& factory)
{
  // Motor Controllers
  constexpr robot::MotorController::Parameter motor_controller_default_parameter{ };
  constexpr std::array<const char*, 4> motor_controller_name = {
    "motor_a", "motor_b", "motor_c", "motor_d"};
  // \todo fix the wrong order of joints!
  constexpr std::array<const char*, 4> motor_controller_joint_name = {
    "joint_wheel_back_right", "joint_wheel_front_right", "joint_wheel_back_left",
    "joint_wheel_front_left"};

  for (std::size_t i = 0; i < motor_controller_name.size(); ++i) {
    const auto motor_controller_parameter = robot::MotorController::get_parameter(
      motor_controller_name[i], motor_controller_default_parameter);
    registerMotorController(std::make_shared<robot::MotorController>(
      motor_controller_name[i],
      i,
      motor_controller_parameter,
      motor_controller_joint_name[i],
      factory.motorControllerHardware().at(motor_controller_name[i]),
      factory.motorSensorHardware().at(motor_controller_name[i])
      ));
    factory.motorControllerHardware().at(motor_controller_name[i])->initialize(motor_controller_parameter);
    factory.motorSensorHardware().at(motor_controller_name[i])->initialize(motor_controller_parameter);
    switchKinematic(Mode::MECANUM_DRIVE);   
    }
}
Eigen::MatrixXf Ohmnibot::getKinematicMatrix(const Mode mode) 
{
  Eigen::MatrixXf kinematic_matrix;

  if (mode & Mode::MECANUM_DRIVE) {
    const float l_x = _parameter.mecanum.length.x;
    const float l_y = _parameter.mecanum.length.y;
    const float wheel_radius = _parameter.mecanum.wheel_diameter * 0.5f;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix <<  1.0f, -1.0f, (l_x + l_y) * 0.5f,
                         1.0f,  1.0f, (l_x + l_y) * 0.5f,
                        -1.0f, -1.0f, (l_x + l_y) * 0.5f,
                        -1.0f,  1.0f, (l_x + l_y) * 0.5f;
    kinematic_matrix *= 1.0f / wheel_radius;    
  }
  else {
    throw std::invalid_argument("Eduard: given kinematic is not supported.");
  }
  return kinematic_matrix;
}
Ohmnibot::~Ohmnibot()
{

}

} // end namespace ohmnibot
} // end namespace robot
} // end namespace eduart
