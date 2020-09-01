#ifndef LORA_PANEL_H
#define LORA_PANEL_H

#ifndef Q_MOC_RUN
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>
#include <ros/ros.h>
#include <rviz/panel.h>
#include "lora_robot.h"
#endif

namespace darpa_dashboard
{

class LoraPanel: public rviz::Panel
{
Q_OBJECT
public:
  LoraPanel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void drop();
  void setR(int);

protected:
  QWidget* lora_widget;

  QLabel* robot_qlabel;

  int active_robot;

  QPushButton** r_qbuttons;
  QPushButton* drop_lora_qbutton;

  LoraRobot** robots;
};

}

#endif 
