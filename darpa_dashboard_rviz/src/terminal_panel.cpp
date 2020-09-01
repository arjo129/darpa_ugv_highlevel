#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>

#include "rosgraph_msgs/Log.h"
#include "terminal_panel.h"
#include "terminal_robot.h"
#include "constants.h"

namespace darpa_dashboard
{

TerminalPanel::TerminalPanel( QWidget* parent ): 
  rviz::Panel( parent )
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  terminal = new QTextEdit();
  terminal->setReadOnly(true);

  topic_layout->addWidget(terminal);

  terminal_widget = new QWidget;

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

  terminal_widget->setEnabled( false );

  robots = new TerminalRobot*[NUMBER_OF_ROBOTS];
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    robots[i] = new TerminalRobot(i+1);
    connect(robots[i], &TerminalRobot::emitCo2, this, &TerminalPanel::receivedCo2);
    connect(robots[i], &TerminalRobot::emitWifi, this, &TerminalPanel::receivedWifi);
  }

  rosout_subscriber = nh.subscribe<rosgraph_msgs::Log> ("/rosout", 5, &TerminalPanel::rosoutCallback, this);
}

void TerminalPanel::receivedCo2(std::string log)
{
  QString log_qstring = QString::fromStdString(log);
  terminal->append(log_qstring);
}

void TerminalPanel::receivedWifi(std::string log)
{
  QString log_qstring = QString::fromStdString(log);
  terminal->append(log_qstring);
}

void TerminalPanel::rosoutCallback(const rosgraph_msgs::Log::ConstPtr& msg)
{
  std::stringstream log;
  log << "[" << msg->header.stamp << "]" << "   " << msg->msg << "   " << msg->function;
  QString log_qstring = QString::fromStdString(log.str());
  terminal->append(log_qstring);
}

void TerminalPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void TerminalPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darpa_dashboard::TerminalPanel,rviz::Panel )
// END_TUTORIAL
