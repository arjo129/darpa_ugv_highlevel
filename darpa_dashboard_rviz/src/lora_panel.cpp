#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>

#include "std_msgs/String.h"
#include "lora_robot.h"
#include "lora_panel.h"
#include "constants.h"

namespace darpa_dashboard
{
LoraPanel::LoraPanel(QWidget* parent): 
  rviz::Panel(parent),
  active_robot(0)
{
  QHBoxLayout* robot_layout = new QHBoxLayout;
  robot_layout->addWidget(new QLabel("Robot:"));
  robot_qlabel = new QLabel(QString::fromStdString(std::to_string(active_robot+1)));
  robot_layout->addWidget(robot_qlabel);

  QHBoxLayout* robots_layout = new QHBoxLayout;
  robots = new LoraRobot*[NUMBER_OF_ROBOTS];
  r_qbuttons = new QPushButton*[NUMBER_OF_ROBOTS];
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    robots[i] = new LoraRobot(i+1);
    r_qbuttons[i] = new QPushButton(QString::fromStdString("R"+std::to_string(i+1)));
    robots_layout->addWidget(r_qbuttons[i]);
    connect(r_qbuttons[i], &QPushButton::pressed, [=]{setR(i);});
  }

  QHBoxLayout* drop_lora_layout = new QHBoxLayout;
  drop_lora_qbutton = new QPushButton("Drop");
  drop_lora_layout->addWidget(drop_lora_qbutton);
  connect(drop_lora_qbutton, SIGNAL(pressed()), this, SLOT(drop()));

  // Create panel.
  lora_widget = new QWidget;

  // Arrange layout in panel
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(robots_layout);
  layout->addLayout(robot_layout);
  layout->addLayout(drop_lora_layout);
  setLayout( layout );

  lora_widget->setEnabled( false );
}

void LoraPanel::drop() {
  robots[active_robot]->drop();
}

void LoraPanel::setR(int i)
{
  active_robot = i;
  robot_qlabel->setText(QString::fromStdString(std::to_string(active_robot+1)));
}


void LoraPanel::save(rviz::Config config) const
{
  rviz::Panel::save( config );
}

void LoraPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darpa_dashboard::LoraPanel,rviz::Panel )
// END_TUTORIAL
