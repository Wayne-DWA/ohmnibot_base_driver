#pragma once

#include "edu_robot/hardware_component_factory.hpp"
#include "edu_robot/motor_controller.hpp"
#include "edu_robot/robot_hardware_interface.hpp"
#include "edu_robot/sensor.hpp"
#include "edu_robot/mode.hpp"
#include "edu_robot/processing_component/collison_avoidance.hpp"
#include "edu_robot/processing_component/processing_detect_charging.hpp"
#include "edu_robot/processing_component/odometry_estimator.hpp"

#include "edu_robot/msg/mode.hpp"
#include "edu_robot/msg/set_lighting_color.hpp"
#include "edu_robot/msg/robot_status_report.hpp"
#include "edu_robot/srv/set_mode.hpp"
#include "edu_robot/srv/get_kinematic_description.hpp"

#include <Eigen/Core>

#include <Eigen/src/Core/Matrix.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// #include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

// #include <rclcpp/node.hpp>
// #include <rclcpp/publisher.hpp>
// #include <rclcpp/service.hpp>
// #include <rclcpp/subscription.hpp>
// #include <rclcpp/timer.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/Twist.h"

#include <memory>
#include <map>
#include <string>

namespace eduart {
namespace robot {
namespace ohmnibot {

/**
 * \brief Defines and implements basic functionality of robot in Eduart universe. Each robot inherits from this
 *        class has to implement a hardware driver that is used for hardware abstraction.
 */
class Ohmnibot
{
protected:
  Ohmnibot();

public:
  struct Parameter {
    std::string tf_base_frame = "base_link";
    std::string tf_footprint_frame = "base_footprint";
    std::string sub_twist_topic = "cmd_vel";
    std::string pub_odom_topic = "odometry";
    std::string topic_joint_states = "base_joint_states";

    double timeout_cmd_vel = 0.5;
    bool enable_collision_avoidance = true;
    bool publish_tf_odom = true;
    bool enable_joint_state_publisher = true;

    struct {
      struct {
        float x = 0.25f;
        float y = 0.32f;
      } length;
      float wheel_diameter = 0.17f;
    } mecanum;
  };

  virtual ~Ohmnibot();

protected:
  /**
   * \brief Initializes this robot by using the provided hardware component factory. The factory has to provide the hardware realizations
   *        needed by an specific robot.
   * \param factory Hardware component factory that must provided hardware interfaces.
   */
  virtual void initialize(eduart::robot::HardwareComponentFactory& factory) = 0;
  /**
   * \brief Callback used to process received twist messages by passing it to the hardware abstraction layer.
   *
   * \param twist_msg Received twist message.
   */
  void cbCmdVel(const geometry_msgs::Twist::ConstPtr& twist_msg);
  // void callbackServiceGetKinematicDescription(
  //   const std::shared_ptr<edu_robot::srv::GetKinematicDescription::Request> request,
  //   std::shared_ptr<edu_robot::srv::GetKinematicDescription::Response> response);
  // Configuration Methods
  void registerMotorController(std::shared_ptr<MotorController> motor_controller);
  // Each robot must provide a kinematic matrix based on given mode.
  virtual Eigen::MatrixXf getKinematicMatrix(const Mode mode) const = 0;
  void switchKinematic(const Mode mode);
  inline std::shared_ptr<tf2_ros::TransformBroadcaster> getTfBroadcaster() { return _tf_broadcaster; }
  std::string getFrameIdPrefix() const;
  bool loadParameter();

  // Parameter
  Parameter _parameter;

  // Hardware Interface
  std::shared_ptr<RobotHardwareInterface> _hardware_interface;

  // Drive Kinematic
  Eigen::MatrixXf _kinematic_matrix;
  Eigen::MatrixXf _inverse_kinematic_matrix;

  // Processing Components
  std::shared_ptr<processing::OdometryEstimator> _odometry_component;


  // Mounted components that are controlled by the hardware interface.
  std::map<std::uint8_t, std::shared_ptr<MotorController>> _motor_controllers;

private:

  // ROS related members
  ros::NodeHandle _nh;
    //ros::ServiceServer _srv_set_mode;

   ros::Publisher _pub_odometry;
   ros::Publisher _pub_status_report;
   ros::Publisher _pub_kinematic_description;
   ros::Publisher _pub_joint_state;
   ros::Subscriber _sub_twist;
   std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
   ros::Time _stamp_cmd_vel;

  std::string _logger_prefix;


//   std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _pub_odometry;
//   std::shared_ptr<rclcpp::Publisher<edu_robot::msg::RobotStatusReport>> _pub_status_report;
//   std::shared_ptr<rclcpp::Publisher<edu_robot::msg::RobotKinematicDescription>> _pub_kinematic_description;

//   std::shared_ptr<rclcpp::Service<edu_robot::srv::SetMode>> _srv_set_mode;

//   std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_twist;



  // Timer used for synchronous processing
//   std::shared_ptr<rclcpp::TimerBase> _timer_status_report;
//   std::shared_ptr<rclcpp::TimerBase> _timer_tf_publishing;
//   std::shared_ptr<rclcpp::TimerBase> _timer_watch_dog;
};
} // end namespace ohmnibot
} // end namespace robot
} // end namespace eduart
