#include <rviz/panel.h>
#include "terminal_robot.h"
#include "std_msgs/String.h"
#include "wireless_msgs/Co2.h"
#include "wireless_msgs/WifiArray.h"

namespace darpa_dashboard
{
TerminalRobot::TerminalRobot(int i):
  robot_id(i)
{
  std::stringstream co2_topic;
  co2_topic << "/X" << robot_id << "/co2";
  co2_subscriber = nh.subscribe(co2_topic.str(), 5, &TerminalRobot::co2Callback, this);
  std::stringstream wifi_topic;
  wifi_topic << "/X" << robot_id << "/wifi";
  wifi_subscriber = nh.subscribe(wifi_topic.str(), 5, &TerminalRobot::wifiCallback, this);
}

void TerminalRobot::co2Callback(const wireless_msgs::Co2& msg)
{
  std::string log = "Seq No: " + std::to_string(msg.header.seq) + ", CO2-Conc: " + std::to_string(msg.concentration) + "at position " + std::to_string(msg.position.x) + ", " + std::to_string(msg.position.y) + ", " + std::to_string(msg.position.z);
  Q_EMIT emitCo2(log);
}

void TerminalRobot::wifiCallback(const wireless_msgs::WifiArray& msg)
{
  for (int i = 0; i < msg.data.size(); i++) {
    wireless_msgs::Wifi wifi = msg.data[i];
    std::string log = "Seq No: " + std::to_string(msg.header.seq) + ", SSID: " + std::string(wifi.ssid.data.c_str()) + ", Signal: " + std::string(wifi.signal.data.c_str()) + "at position " + std::to_string(msg.position.x) + ", " + std::to_string(msg.position.y) + ", " + std::to_string(msg.position.z);
    Q_EMIT emitWifi(log);
  }
}

}