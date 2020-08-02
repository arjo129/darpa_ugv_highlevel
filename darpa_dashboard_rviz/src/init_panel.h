#ifndef INIT_PANEL_H
#define INIT_PANEL_H

#ifndef Q_MOC_RUN
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <rviz/panel.h>
#include "init_robot.h"
#endif

namespace darpa_dashboard
{

class InitPanel: public rviz::Panel
{
Q_OBJECT
public:
  InitPanel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void setR(int);
  void start();
  void stop();
  void startAll();
  void stopAll();

protected:
  QWidget* init_widget;

  QLabel* robot_qlabel;

  QPushButton** r_qbuttons;
  QPushButton* stop_qbutton;
  QPushButton* start_qbutton;
  QPushButton* stop_all_qbutton;
  QPushButton* start_all_qbutton;

  int active_robot;

  InitRobot** robots;
};

}

#endif 
