#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QRegExpValidator>

#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "artifact_robot.h"
#include "artifact_panel.h"
#include "constants.h"

namespace darpa_dashboard
{
ArtifactPanel::ArtifactPanel(QWidget* parent): 
  rviz::Panel(parent),
  active_robot(0)
{
  QHBoxLayout* robot_layout = new QHBoxLayout;
  robot_layout->addWidget(new QLabel("Robot:"));
  robot_qlabel = new QLabel(QString::fromStdString(std::to_string(active_robot+1)));
  robot_layout->addWidget(robot_qlabel);

  QHBoxLayout* robots_layout = new QHBoxLayout;
  robots = new ArtifactRobot*[NUMBER_OF_ROBOTS];
  r_qbuttons = new QPushButton*[NUMBER_OF_ROBOTS];
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    robots[i] = new ArtifactRobot(i+1);
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

  QHBoxLayout* goal_z_layout = new QHBoxLayout;
  goal_z_layout->addWidget(new QLabel("z"));
  goal_z_qedit = new QLineEdit;
  goal_z_qedit->setText("0");
  goal_z_qedit->setValidator(rxv);
  goal_z_layout->addWidget(goal_z_qedit);

  QHBoxLayout* dropdown_menu_layout = new QHBoxLayout;
  QStringList commands = { "Survivor", "Cell Phone", "Backpack", "Drill", "Fire Extinguisher", "Gas", "Vent" };
  combo = new QComboBox;
  combo->addItems(commands);
  dropdown_menu_layout->addWidget(combo);

  QHBoxLayout* drop_artifact_layout = new QHBoxLayout;
  drop_artifact_qbutton = new QPushButton("Report");
  drop_artifact_layout->addWidget(drop_artifact_qbutton);
  connect(drop_artifact_qbutton, SIGNAL(pressed()), this, SLOT(drop()));

  // Create panel.
  artifact_widget = new QWidget;

  // Arrange layout in panel
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(robots_layout);
  layout->addLayout(robot_layout);
  layout->addLayout(goal_x_layout);
  layout->addLayout(goal_y_layout);
  layout->addLayout(goal_z_layout);
  layout->addLayout(dropdown_menu_layout);
  layout->addLayout(drop_artifact_layout);
  setLayout( layout );

  artifact_widget->setEnabled( false );

  clicked_point_subscriber = nh.subscribe<geometry_msgs::PointStamped> ("/clicked_point", 5, &ArtifactPanel::callback, this);
}

void ArtifactPanel::callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  goal_x_qedit->setText(QString::fromStdString(std::to_string(msg->point.x)));
  goal_y_qedit->setText(QString::fromStdString(std::to_string(msg->point.y)));
  goal_z_qedit->setText(QString::fromStdString(std::to_string(msg->point.z)));
}

void ArtifactPanel::drop() {
  std::string x = goal_x_qedit->text().toStdString();
  std::string y = goal_y_qedit->text().toStdString();
  std::string z = goal_z_qedit->text().toStdString();
  std::string artifact_type = combo->currentText().toStdString();
  robots[active_robot]->drop(stod(x), stod(y), stod(z), artifact_type);
}

void ArtifactPanel::setR(int i)
{
  active_robot = i;
  robot_qlabel->setText(QString::fromStdString(std::to_string(active_robot+1)));
}


void ArtifactPanel::save(rviz::Config config) const
{
  rviz::Panel::save( config );
}

void ArtifactPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darpa_dashboard::ArtifactPanel,rviz::Panel )
// END_TUTORIAL
