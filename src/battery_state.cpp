#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/BatteryState.h"

class BatteryState
{
protected:
  // Subscriber to thrusters use in percent
  void ReadThrustersUse(const sensor_msgs::JointStateConstPtr& _msg)
  {
    thrusters_use_percent_ = *_msg;
  }

public:
  BatteryState(ros::NodeHandle &ros_node)
  {
    // init publishers
    battery_state_publisher_ = ros_node.advertise<sensor_msgs::BatteryState>("/my_robot/battery_state", 1);

    // battery present and full
    battery_msg_.percentage = 100;
    battery_msg_.present = 1;

    // init subscribers
    thruster_use_subscriber_ = ros_node.subscribe("/my_robot/thruster_use", 1, &BatteryState::ReadThrustersUse, this);
  }

  // Функция считает удельный расход заряда
  // батареи в секунду и вычитает из имеющегося
  void calculateBatteryCondition()
  {
    const float k = 0.0001; // коэффициент пропорциональности
    float p = 0; // текущая относительная мощность двигателей

    for(unsigned int i = 0; i < thrusters_use_percent_.position.size(); i++)
      p += thrusters_use_percent_.position[i]/100;

    if (!thrusters_use_percent_.position.empty() && battery_msg_.percentage > 0)
    {
      battery_msg_.percentage -= (100 * k * p /thrusters_use_percent_.position.size());
    }
  }

  void Publish()
  {
    battery_state_publisher_.publish(battery_msg_);
  }

protected:
  // publishers and messages to be published
  ros::Publisher battery_state_publisher_;
  sensor_msgs::BatteryState battery_msg_;

  // subscribers and received messages
  ros::Subscriber thruster_use_subscriber_;
  sensor_msgs::JointState thrusters_use_percent_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_condition");
  ros::NodeHandle ros_node;

  ros::Rate loop_rate(1); // 1 hz

  // Battery state class
  BatteryState batteryState(ros_node);

  while (ros::ok())
  {
    batteryState.calculateBatteryCondition();
    batteryState.Publish();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
