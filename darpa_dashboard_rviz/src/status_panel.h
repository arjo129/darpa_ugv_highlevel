#ifndef STATUS_PANEL_H
#define STATUS_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#include "std_msgs/String.h"
#endif

class QLabel;

namespace darpa_dashboard
{

class StatusPanel: public rviz::Panel
{
Q_OBJECT
public:
  StatusPanel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void getStatus();

protected:
  QWidget* status_widget;

  QLabel* team_name_qlabel;
  QLabel* current_time_qlabel;
  QLabel* reports_left_qlabel;
  QLabel* score_qlabel;

  QString team_name;
  QString current_time;
  QString reports_left;
  QString score;

  ros::ServiceClient status_service_client;

  ros::NodeHandle nh;
};

}

#endif 
