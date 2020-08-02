#ifndef TERMINAL_PANEL_H
#define TERMINAL_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include "rosgraph_msgs/Log.h"
#include "terminal_robot.h"
#endif

class QTextEdit;

namespace darpa_dashboard
{

class TerminalPanel: public rviz::Panel
{
Q_OBJECT
public:
  TerminalPanel( QWidget* parent = 0 );
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

protected Q_SLOTS:
  void rosoutCallback(const rosgraph_msgs::Log::ConstPtr& msg);
  void receivedCo2(std::string);
  void receivedWifi(std::string);
  
protected:
  QWidget* terminal_widget;

  QTextEdit* terminal;

  ros::Subscriber rosout_subscriber;
  ros::Subscriber clicked_point_subscriber;

  ros::NodeHandle nh;

  TerminalRobot** robots;
};

}

#endif // TELEOP_PANEL_H
