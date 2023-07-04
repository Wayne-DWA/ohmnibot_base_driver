#include "edu_robot/ethernet_gateway/hardware_component_factory.hpp"
#include <ohmnibot_base_driver/ohmnibot_base_driver.h>
#include <edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp>

#include <memory>
#include "ros/ros.h"

class EthernetGatewayOhmniBot : public eduart::robot::ohmnibot::Ohmnibot
{
public:
  EthernetGatewayOhmniBot()
    : eduart::robot::ohmnibot::Ohmnibot(
        "ohmnibot",
        std::make_unique<eduart::robot::ethernet::EthernetGatewayShield>("192.168.2.20", 1234)
      )
  {
    auto shield = std::dynamic_pointer_cast<eduart::robot::ethernet::EthernetGatewayShield>(_hardware_interface);
    shield->enable();
    auto factory = eduart::robot::ethernet::HardwareComponentFactory(shield);

    factory.addSingleChannelMotorController("motor", "motor_hardware");
          //  .addImuSensor("imu", "imu_hardware", *this);

    initialize(factory);
  }

  void odomPublisher()
  {
    pubOdom();
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv,"Ohmnibot_Base_Driver_Node");

    EthernetGatewayOhmniBot egob;
    ros::Rate loop_rate_hz(50);
    while(ros::ok())
    {
        egob.odomPublisher();
        ros::spinOnce();
        loop_rate_hz.sleep();
    }
    exit(0);
}
