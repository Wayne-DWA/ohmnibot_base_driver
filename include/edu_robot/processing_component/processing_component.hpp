/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

// #include <rclcpp/clock.hpp>
// #include <rclcpp/node.hpp>
// #include <rclcpp/time.hpp>
#include "ros/ros.h"
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

namespace eduart {
namespace robot {
namespace processing {

template <typename Output>
class ProcessingComponentOutput;

template <typename Input>
class ProcessingComponentInput
{
public:
  virtual ~ProcessingComponentInput() = default;

  virtual void processInput(const Input& value, const ProcessingComponentOutput<Input>* sender) = 0;
};

template <typename Output>
class ProcessingComponentOutput
{
public:
  ProcessingComponentOutput(const std::string& name) : _name(name) { }
  virtual ~ProcessingComponentOutput() = default;

  const std::string& name() const { return _name; }
  void registerComponentInput(std::shared_ptr<ProcessingComponentInput<Output>> input)
  {
    if (std::find(_inputs.begin(), _inputs.end(), input) != _inputs.end()) {
      throw std::invalid_argument("Given component input is already registered. Can't add it twice.");      
    }

    _inputs.push_back(input);
  }

protected:
  void sendInputValue(const Output& value)
  {
    for (auto& input : _inputs) {
      input->processInput(value, this);
    }
  }

private:
  std::string _name;
  std::vector<std::shared_ptr<ProcessingComponentInput<Output>>> _inputs;
};

class ProcessingComponent
{
protected:
  ProcessingComponent(const std::string& name)
    : _name(name)
    , _last_processing(ros::Time::now())
  { }

public:
  ProcessingComponent(const ProcessingComponent&) = delete;
  virtual ~ProcessingComponent() = default;

  const std::string& name() const { return _name; }

private:
  std::string _name;

protected:
  // std::shared_ptr<rclcpp::Clock> _clock;
  ros::Time _last_processing;
};

} // end namespace processing
} // end namespace robot
} // end namespace eduart
