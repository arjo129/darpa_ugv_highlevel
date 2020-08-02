#ifndef TERMINAL_ROBOT_H
#define TERMINAL_ROBOT_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QObject>
#include "std_msgs/String.h"
#include "wireless_msgs/Co2.h"
#include "wireless_msgs/WifiArray.h"
#endif

namespace darpa_dashboard
{

class TerminalRobot: public QObject
{
Q_OBJECT
public:
  TerminalRobot(int);
  void co2Callback(const wireless_msgs::Co2& msg);
  void wifiCallback(const wireless_msgs::WifiArray& msg);

Q_SIGNALS:
  void emitCo2(std::string);
  void emitWifi(std::string);

private:
  int robot_id;

  ros::Subscriber co2_subscriber;
  ros::Subscriber wifi_subscriber;

  ros::NodeHandle nh;
};

}

#endif 
