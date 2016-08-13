#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

std::vector<int> thruster_use_percent;

/*
 * Функция считает удельный расход заряда батареи в секунду
 */
float calculateBatteryCondition()
{
  float k = 0.0001; // коэффициент пропорциональности
  float p = 0; // текущая относительная мощность двигателя

  for(unsigned int i = 0; i < thruster_use_percent.size(); i++)
    p += (float) thruster_use_percent[i]/100;

  return k * p / thruster_use_percent.size();
}

/*
 * Функция читает топик /my_robot/thruster_use
 * и записывает значения в thruster_use_percent
 */
void thrusterUseCallback(const sensor_msgs::JointStateConstPtr & _msg)
{
  unsigned int i;

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_condition");
  ros::NodeHandle rosnode;

  ros::Subscriber thruster_use_subscriber = rosnode.subscribe("/my_robot/thruster_use", 1, thrusterUseCallback);

  ros::Rate loop_rate(1); // 1 hz
  while (ros::ok())
  {
    ROS_INFO("I heard [%f]", calculateBatteryCondition());
    /*
     * Сделать пересчет заряда батареи в процентах
     * и постить в battery_state топик
     * http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
     */

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
