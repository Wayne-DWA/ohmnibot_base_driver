#include "edu_robot/motor_controller.hpp"
#include "edu_robot/rotation_per_minute.hpp"

#include <cstdint>
#include <functional>
#include <thread>
#include <mutex>
#include "ros/ros.h"

namespace eduart {
namespace robot {

MotorController::Parameter MotorController::get_parameter(const std::string& name, const MotorController::Parameter& default_parameter)
{
    std::string prefix = name;
    std::replace(prefix.begin(), prefix.end(), '/', '.');
    MotorController::Parameter parameter;

    // ROS_INFO(_logger_prefix <<"get parameters from server");
    ros::NodeHandle privateNh("~");

    privateNh.param(prefix + ".inverted", parameter.inverted, default_parameter.inverted);
    privateNh.param(prefix + ".gear_ratio", parameter.gear_ratio,default_parameter.gear_ratio);
    privateNh.param(prefix + ".encoder_ratio", parameter.encoder_ratio, default_parameter.encoder_ratio);
    privateNh.param(prefix + ".max_rpm", parameter.max_rpm, default_parameter.max_rpm);
    privateNh.param(prefix + ".control_frequency", parameter.control_frequency, default_parameter.control_frequency);
    privateNh.param(prefix + ".pid.kp", parameter.kp, default_parameter.kp);
    privateNh.param(prefix + ".pid.ki", parameter.ki, default_parameter.ki);
    privateNh.param(prefix + ".pid.kd", parameter.kd, default_parameter.kd);
    privateNh.param(prefix + ".weight_low_pass_set_point", parameter.weight_low_pass_set_point, default_parameter.weight_low_pass_set_point);
    privateNh.param(prefix + ".weight_low_pass_encoder", parameter.weight_low_pass_encoder, default_parameter.weight_low_pass_encoder);
    privateNh.param(prefix + ".encoder_inverted", parameter.encoder_inverted, default_parameter.encoder_inverted);
    privateNh.param(prefix + ".closed_loop", parameter.closed_loop, default_parameter.closed_loop);
    return parameter;
}

MotorController::MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter,
                                 const std::string& urdf_joint_name,
                                 std::shared_ptr<ComponentInterface> hardware_component_interface,
                                 std::shared_ptr<SensorInterface> hardware_sensor_interface)
  : _parameter(parameter)
  , _name(name)
  , _id(id)
  , _urdf_joint_name(urdf_joint_name)
  , _stamp_last_measurement(ros::Time::now())
  , _current_wheel_position(0.0)
  , _hardware_component_interface(hardware_component_interface)
  , _hardware_sensor_interface(hardware_sensor_interface)
{
  _hardware_sensor_interface->registerCallbackProcessMeasurementData(
    std::bind(&MotorController::processMeasurementData, this, std::placeholders::_1)
  );
  _pub_joint_state = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
}

MotorController::~MotorController()
{

}

void MotorController::setRpm(const Rpm rpm)
{
  _hardware_component_interface->processSetValue(rpm);
  _set_rpm = rpm;
}

void MotorController::processMeasurementData(const Rpm rpm)
{
  {
    std::lock_guard guard(_mutex_access_data);
    _measured_rpm = rpm;
  }

  // \todo Check if calculation is correct! At the moment used for visualization only, so no need for accurate calc...
  // perform wheel position calculation
  const auto stamp = ros::Time::now();
  const auto delta_t = stamp - _stamp_last_measurement;

  _current_wheel_position += delta_t.toSec() * rpm.radps();
  _stamp_last_measurement = stamp;

  // publish wheel position as tf message
  // geometry_msgs::msg::TransformStamped msg;

  // publish wheel speed using joint state message
  // sensor_msgs::msg::JointState joint_state_msg;
  sensor_msgs::JointState joint_state_msg;

  joint_state_msg.header.frame_id = "";
  joint_state_msg.header.stamp = stamp;
  joint_state_msg.name.push_back(_urdf_joint_name);
  joint_state_msg.velocity.push_back(rpm.radps());
  joint_state_msg.position.push_back(_current_wheel_position);

  // _pub_joint_state->publish(joint_state_msg);
  _pub_joint_state.publish(joint_state_msg);
}

} // end namespace robot
} // end namespace eduart
