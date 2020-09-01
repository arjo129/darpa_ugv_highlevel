#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRegExpValidator>
#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "twist_panel.h"
#include "twist_robot.h"
#include "constants.h"

namespace darpa_dashboard
{
TwistPanel::TwistPanel(QWidget* parent): 
  rviz::Panel(parent), 
  active_robot(0),
  y_velocity(0),
  x_velocity(0),
  yaw_velocity(0),
  is_reset(true)
{
  QHBoxLayout* robot_layout = new QHBoxLayout;
  robot_layout->addWidget(new QLabel("Robot:"));
  robot_qlabel = new QLabel(QString::fromStdString(std::to_string(active_robot+1)));
  robot_layout->addWidget(robot_qlabel);

  QHBoxLayout* robots_layout = new QHBoxLayout;
  robots = new TwistRobot*[NUMBER_OF_ROBOTS];
  r_qbuttons = new QPushButton*[NUMBER_OF_ROBOTS];
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    robots[i] = new TwistRobot(i+1);
    r_qbuttons[i] = new QPushButton(QString::fromStdString("R"+std::to_string(i+1)));
    robots_layout->addWidget(r_qbuttons[i]);
    connect(r_qbuttons[i], &QPushButton::pressed, [=]{setR(i);});
  }

  QRegExpValidator* rxv = new QRegExpValidator(QRegExp("[+-]?\\d*\\.\\d*"), this); // pos and neg

  QHBoxLayout* goal_x_layout = new QHBoxLayout;
  goal_x_layout->addWidget(new QLabel("x"));
  goal_x_qedit = new QLineEdit;
  goal_x_qedit->setText("0");
  goal_x_qedit->setValidator(rxv);
  goal_x_layout->addWidget(goal_x_qedit);

  QHBoxLayout* goal_y_layout = new QHBoxLayout;
  goal_y_layout->addWidget(new QLabel("y"));
  goal_y_qedit = new QLineEdit;
  goal_y_qedit->setText("0");
  goal_y_qedit->setValidator(rxv);
  goal_y_layout->addWidget(goal_y_qedit);

  QHBoxLayout* goal_yaw_layout = new QHBoxLayout;
  goal_yaw_layout->addWidget(new QLabel("θ"));
  goal_yaw_qedit = new QLineEdit;
  goal_yaw_qedit->setText("0");
  goal_yaw_qedit->setValidator(rxv);
  goal_yaw_layout->addWidget(goal_yaw_qedit);

  QHBoxLayout* send_goal_layout = new QHBoxLayout;
  send_goal_qbutton = new QPushButton("Move");
  send_goal_layout->addWidget(send_goal_qbutton);
  connect(send_goal_qbutton, SIGNAL(pressed()), this, SLOT(move()));

  QHBoxLayout* p1_layout = new QHBoxLayout;
  p1_x_qlabel = new QLabel(QString::number(p1.x));
  p1_y_qlabel = new QLabel(QString::number(p1.y));
  p1_z_qlabel = new QLabel(QString::number(p1.z));
  p1_layout->addWidget(new QLabel("p1"));
  p1_layout->addWidget(new QLabel(": "));
  p1_layout->addWidget(p1_x_qlabel);
  p1_layout->addWidget(new QLabel(", "));
  p1_layout->addWidget(p1_y_qlabel);
  p1_layout->addWidget(new QLabel(", "));
  p1_layout->addWidget(p1_z_qlabel);

  QHBoxLayout* p2_layout = new QHBoxLayout;
  p2_x_qlabel = new QLabel(QString::number(p2.x));
  p2_y_qlabel = new QLabel(QString::number(p2.y));
  p2_z_qlabel = new QLabel(QString::number(p2.z));
  p2_layout->addWidget(new QLabel("p2"));
  p2_layout->addWidget(new QLabel(": "));
  p2_layout->addWidget(p2_x_qlabel);
  p2_layout->addWidget(new QLabel(", "));
  p2_layout->addWidget(p2_y_qlabel);
  p2_layout->addWidget(new QLabel(", "));
  p2_layout->addWidget(p2_z_qlabel);

  QHBoxLayout* reset_layout = new QHBoxLayout;
  reset_qbutton = new QPushButton("Reset");
  reset_layout->addWidget(reset_qbutton);
  connect(reset_qbutton, SIGNAL(pressed()), this, SLOT(reset()));

  // Create panel.
  twist_widget = new QWidget;

  // Arrange layout in panel
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(robots_layout);
  layout->addLayout(p1_layout);
  layout->addLayout(p2_layout);
  layout->addLayout(robot_layout);
  layout->addLayout(goal_x_layout);
  layout->addLayout(goal_y_layout);
  layout->addLayout(goal_yaw_layout);
  layout->addLayout(send_goal_layout);
  layout->addLayout(reset_layout);
  setLayout(layout);

  twist_widget->setEnabled( false );

  clicked_point_subscriber = nh.subscribe<geometry_msgs::PointStamped> ("/clicked_point", 5, &TwistPanel::callback, this);
}

void TwistPanel::reset() {
    p1.x  = 0;
    p1.y = 0;
    p1.z = 0;
    p1_x_qlabel->setText(QString::number(0));
    p1_y_qlabel->setText(QString::number(0));
    p1_z_qlabel->setText(QString::number(0));
    p2.x = 0;
    p2.y = 0;
    p2.z = 0;
    p2_x_qlabel->setText(QString::number(0));
    p2_y_qlabel->setText(QString::number(0));
    p2_z_qlabel->setText(QString::number(0));
    is_reset = true;
}

void TwistPanel::callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  if (is_reset)
  {
    p1.x  = msg->point.x;
    p1.y = msg->point.y;
    p1.z = msg->point.z;
    p1_x_qlabel->setText(QString::number(p1.x));
    p1_y_qlabel->setText(QString::number(p1.y));
    p1_z_qlabel->setText(QString::number(p1.z));
    is_reset = false;
  }
  else {
    p2.x = msg->point.x;
    p2.y = msg->point.y;
    p2.z = msg->point.z;
    p2_x_qlabel->setText(QString::number(p2.x));
    p2_y_qlabel->setText(QString::number(p2.y));
    p2_z_qlabel->setText(QString::number(p2.z));
    goal_x_qedit->setText(QString::fromStdString(std::to_string(p2.x - p1.x)));
    goal_y_qedit->setText(QString::fromStdString(std::to_string(p2.y - p1.y)));
    goal_yaw_qedit->setText(QString::fromStdString(std::to_string(p2.z - p1.z)));
  }
}

void TwistPanel::move() 
{
  std::string x = goal_x_qedit->text().toStdString();
  std::string y = goal_y_qedit->text().toStdString();
  std::string yaw = goal_yaw_qedit->text().toStdString();
  robots[active_robot]->move(stod(x), stod(y), stod(yaw));
}


void TwistPanel::setR(int i)
{
  active_robot = i;
  robot_qlabel->setText(QString::fromStdString(std::to_string(active_robot+1)));
}

void TwistPanel::save(rviz::Config config) const
{
  rviz::Panel::save( config );
}

void TwistPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darpa_dashboard::TwistPanel,rviz::Panel )
// END_TUTORIAL
