#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

#include "init_panel.h"
#include "std_msgs/String.h"
#include "init_robot.h"
#include "constants.h"

namespace darpa_dashboard
{
InitPanel::InitPanel(QWidget* parent): 
  rviz::Panel(parent), 
  active_robot(0)
{
  QHBoxLayout* robot_layout = new QHBoxLayout;
  robot_layout->addWidget(new QLabel("Robot:"));
  robot_qlabel = new QLabel(QString::fromStdString(std::to_string(active_robot+1)));
  robot_layout->addWidget(robot_qlabel);

  QHBoxLayout* robots_layout = new QHBoxLayout;
  robots = new InitRobot*[NUMBER_OF_ROBOTS];
  r_qbuttons = new QPushButton*[NUMBER_OF_ROBOTS];
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    robots[i] = new InitRobot(i+1);
    r_qbuttons[i] = new QPushButton(QString::fromStdString("R"+std::to_string(i+1)));
    robots_layout->addWidget(r_qbuttons[i]);
    connect(r_qbuttons[i], &QPushButton::pressed, [=]{setR(i);});
  }

  QHBoxLayout* init_robot_layout = new QHBoxLayout;
  start_qbutton = new QPushButton("Start");
  connect(start_qbutton, SIGNAL(pressed()), this, SLOT(start()));
  stop_qbutton = new QPushButton("Stop");
  connect(stop_qbutton, SIGNAL(pressed()), this, SLOT(stop()));
  init_robot_layout->addWidget(stop_qbutton);
  init_robot_layout->addWidget(start_qbutton);

  QHBoxLayout* init_all_robots_layout = new QHBoxLayout;
  start_all_qbutton = new QPushButton("Start All");
  connect(start_all_qbutton, SIGNAL(pressed()), this, SLOT(startAll()));
  stop_all_qbutton = new QPushButton("Stop All");
  connect(stop_all_qbutton, SIGNAL(pressed()), this, SLOT(stopAll()));
  init_all_robots_layout->addWidget(stop_all_qbutton);
  init_all_robots_layout->addWidget(start_all_qbutton);

  // Create panel.
  init_widget = new QWidget;

  // Arrange layout in panel
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(robots_layout);
  layout->addLayout(robot_layout);
  layout->addLayout(init_robot_layout);
  layout->addLayout(init_all_robots_layout);
  setLayout( layout );

  init_widget->setEnabled( false );
}

void InitPanel::stop() 
{
  robots[active_robot]->stop();
}

void InitPanel::start() 
{
  robots[active_robot]->start();
}

void InitPanel::stopAll() 
{
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    robots[i]->stop();
  }
}

void InitPanel::startAll() 
{
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    robots[i]->start();
  }
}

void InitPanel::save(rviz::Config config) const
{
  rviz::Panel::save( config );
}

void InitPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

void InitPanel::setR(int i)
{
  active_robot = i;
  robot_qlabel->setText(QString::fromStdString(std::to_string(active_robot+1)));
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darpa_dashboard::InitPanel,rviz::Panel )
// END_TUTORIAL
