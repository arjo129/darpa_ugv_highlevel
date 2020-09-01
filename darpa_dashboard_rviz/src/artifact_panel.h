#ifndef ARTIFACT_PANEL_H
#define ARTIFACT_PANEL_H

#ifndef Q_MOC_RUN
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>
#include <ros/ros.h>
#include <rviz/panel.h>
#include "geometry_msgs/PointStamped.h"
#include "artifact_robot.h"
#endif

namespace darpa_dashboard
{

class ArtifactPanel: public rviz::Panel
{
Q_OBJECT
public:
  ArtifactPanel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void drop();
  void setR(int);
  void callback(const geometry_msgs::PointStamped::ConstPtr& msg);

protected:
  QWidget* artifact_widget;

  QLineEdit* goal_x_qedit;
  QLineEdit* goal_y_qedit;
  QLineEdit* goal_z_qedit;

  QComboBox* combo;

  QLabel* robot_qlabel;

  int active_robot;

  QPushButton** r_qbuttons;
  QPushButton* drop_artifact_qbutton;

  ArtifactRobot** robots;

  ros::Subscriber clicked_point_subscriber;

  ros::NodeHandle nh;
};

}

#endif 
