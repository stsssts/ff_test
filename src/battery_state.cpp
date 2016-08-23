#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/BatteryState.h"

std::vector<int> thruster_use_percent;

/*
 * Функция считает удельный расход заряда батареи в секунду
 */
float calculateBatteryCondition()
{
  const float k = 0.0001; // коэффициент пропорциональности
  float p = 0; // текущая относительная мощность двигателя
  for(unsigned int i = 0; i < thruster_use_percent.size() && !thruster_use_percent.empty(); i++)
    p += (float) thruster_use_percent[i]/100;

  return thruster_use_percent.empty() ? 0 : k * p / thruster_use_percent.size();
}

/*
 * Функция читает топик /my_robot/thruster_use
 * и записывает значения в thruster_use_percent
 */
void thrusterUseCallback(const sensor_msgs::JointStateConstPtr & _msg)
{
  unsigned int i;

  if(!_msg->position.empty())
  {
    if(thruster_use_percent.empty())
    {
      for(i = 0; i < _msg->position.size(); i++)
        thruster_use_percent.push_back(_msg->position[i]);
    }
    else
    {
      for(i = 0; i < _msg->position.size(); i++)
        thruster_use_percent[i] = _msg->position[i];
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_condition");
  ros::NodeHandle rosnode;

  ros::Subscriber thruster_use_subscriber = rosnode.subscribe("/my_robot/thruster_use", 1, thrusterUseCallback);
  ros::Publisher battery_state_publisher = rosnode.advertise<sensor_msgs::BatteryState>("/my_robot/battery_state", 1);

  ros::Rate loop_rate(1); // 1 hz

  float battery_filling = 1.0;
  sensor_msgs::BatteryState battery_msg;
  battery_msg.present = true;

  while (ros::ok())
  {
    if(battery_filling > 0)
      battery_filling -= calculateBatteryCondition();
    else
      battery_filling = 0;

    // Publish battery condition
    battery_msg.percentage = battery_filling * 100;
    battery_state_publisher.publish(battery_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
