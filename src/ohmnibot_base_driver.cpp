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

    privateNh.param("mecanum_length_x", _parameter.mecanum.length.x,0.25);
    privateNh.param("mecanum_length_y", _parameter.mecanum.length.y,0.36);
    privateNh.param("mecanum_wheel_diameter", _parameter.mecanum.wheel_diameter,0.1);
    return true;
}
void Ohmnibot::cbCmdVel(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  // Kick Watch Dog
  // _timer_status_report->reset();
      _hardware_interface->enable();

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
    odom_updated =true;

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
void Ohmnibot::pubOdom()
{
    // Estimate delta t
    if(!odom_init) 
    {
      _last_processing = ros::Time::now();
      odom_init = true;
    }
    const auto now = ros::Time::now();
    const double dt = (now - _last_processing).toSec();
    _last_processing = now;
    // Calculating Odometry and Publishing it
    Eigen::VectorXf radps_measured(_motor_controllers.size());
    if(odom_updated)
    {
            for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
        radps_measured(i) = _motor_controllers[i]->getMeasuredRpm().radps();
      }
    }
    else 
    {
      for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
        radps_measured(i) = 0;
      }
    }
    const Eigen::Vector3f velocity_measured = _inverse_kinematic_matrix * radps_measured;
    // ROS_INFO_STREAM(_logger_prefix <<"velocity_measured: x = "
    //                                   << velocity_measured.x()<< ", y = "<<velocity_measured.y());    

    // Processing Velocity
    _linear_velocity_x = velocity_measured.x()*odom_cali_factor_x;
    _linear_velocity_y = velocity_measured.y()*odom_cali_factor_y;
    _angular_velocity_z = velocity_measured.z()*odom_cali_factor_yaw;

    const Eigen::Vector2f liner_velocity(_linear_velocity_x, _linear_velocity_y);
    const Eigen::Vector2f direction = Eigen::Rotation2Df(_orientation) * liner_velocity;

    _orientation += _angular_velocity_z * dt;
    _position_x  += direction.x() * dt;
    _position_y  += direction.y() * dt;



    // Constructing Message
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = ros::Time::now();
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_footprint";

    // Twist Part
    odometry_msg.twist.twist.linear.x = _linear_velocity_x;
    odometry_msg.twist.twist.linear.y = _linear_velocity_y;
    odometry_msg.twist.twist.linear.z = 0.0;

    odometry_msg.twist.twist.angular.x = 0.0;
    odometry_msg.twist.twist.angular.y = 0.0;
    odometry_msg.twist.twist.angular.z = _angular_velocity_z;

    odometry_msg.twist.covariance.fill(0.0);

    // Pose Part
    const Eigen::Quaternionf q_orientation(Eigen::AngleAxisf(_orientation, Eigen::Vector3f::UnitZ()));
    odometry_msg.pose.pose.orientation.w = q_orientation.w();
    odometry_msg.pose.pose.orientation.x = q_orientation.x();
    odometry_msg.pose.pose.orientation.y = q_orientation.y();
    odometry_msg.pose.pose.orientation.z = q_orientation.z();

    odometry_msg.pose.pose.position.x = _position_x;
    odometry_msg.pose.pose.position.y = _position_y;
    odometry_msg.pose.pose.position.z = 0.0;

    odometry_msg.twist.covariance = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,   
                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    // odometry_msg.pose.covariance.fill(0.0);
    
    _pub_odometry.publish(odometry_msg);
    // ROS_DEBUG("odom published");

    if (_parameter.publish_tf_odom) 
    {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.frame_id = "odom";
      tf_msg.header.stamp    = ros::Time::now();
      tf_msg.child_frame_id  = "base_footprint";
      
      tf_msg.transform.translation.x = _position_x;
      tf_msg.transform.translation.y = _position_y;
      tf_msg.transform.translation.z = 0.0;

      const Eigen::Quaternionf q_orientation(Eigen::AngleAxisf(_orientation, Eigen::Vector3f::UnitZ()));
      tf_msg.transform.rotation.x = q_orientation.x();
      tf_msg.transform.rotation.y = q_orientation.y();
      tf_msg.transform.rotation.z = q_orientation.z();
      tf_msg.transform.rotation.w = q_orientation.w();

      _tf_broadcaster.sendTransform(tf_msg);
    }
  odom_updated = false;
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
                        1.0f, 1.0f, -(l_x + l_y) * 0.5f,
                        -1.0f,  1.0f, (l_x + l_y) * 0.5f;
                        // 1 2 3 4
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
