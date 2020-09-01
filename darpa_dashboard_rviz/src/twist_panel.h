#ifndef TWIST_PANEL_H
#define TWIST_PANEL_H

#ifndef Q_MOC_RUN
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <ros/ros.h>
#include <rviz/panel.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "twist_robot.h"
#endif

namespace darpa_dashboard
{

class TwistPanel: public rviz::Panel
{
Q_OBJECT
public:
  TwistPanel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void setR(int);
  void move();
  void reset();
  void callback(const geometry_msgs::PointStamped::ConstPtr& msg);

protected:
  QWidget* twist_widget;

  QLineEdit* goal_x_qedit;
  QLineEdit* goal_y_qedit;
  QLineEdit* goal_yaw_qedit;

  QLabel* robot_qlabel;
  QLabel* p1_x_qlabel;
  QLabel* p1_y_qlabel;
  QLabel* p1_z_qlabel;
  QLabel* p2_x_qlabel;
  QLabel* p2_y_qlabel;
  QLabel* p2_z_qlabel;

  QPushButton* send_goal_qbutton;
  QPushButton** r_qbuttons;
  QPushButton* reset_qbutton;

  float x_velocity;
  float y_velocity;
  float yaw_velocity;

  bool is_reset;

  int active_robot;

  TwistRobot** robots;

  geometry_msgs::Point p1;
  geometry_msgs::Point p2;

  ros::Subscriber clicked_point_subscriber;

  ros::NodeHandle nh;
};

}

#endif 
