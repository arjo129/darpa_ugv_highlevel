#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

#include "status_panel.h"
#include "std_msgs/String.h"
#include "interface_protocol/GetStatus.h"
#include "interface_protocol/GetStatusRequest.h"
#include "interface_protocol/GetStatusResponse.h"

namespace darpa_dashboard
{
StatusPanel::StatusPanel(QWidget* parent): 
  rviz::Panel(parent)
{
  QHBoxLayout* team_name_layout = new QHBoxLayout;
  team_name_layout->addWidget(new QLabel("Team Name:"));
  team_name = QString("NA");
  team_name_qlabel = new QLabel(team_name);
  team_name_layout->addWidget(team_name_qlabel);

  QHBoxLayout* current_time_layout = new QHBoxLayout;
  current_time_layout->addWidget(new QLabel("Current Time:"));
  current_time = QString("NA");
  current_time_qlabel = new QLabel(current_time);
  current_time_layout->addWidget(current_time_qlabel);

  QHBoxLayout* reports_left_layout = new QHBoxLayout;
  reports_left_layout->addWidget(new QLabel("Reports Left:"));
  reports_left = QString("NA");
  reports_left_qlabel = new QLabel(reports_left);
  reports_left_layout->addWidget(reports_left_qlabel);

  QHBoxLayout* score_layout = new QHBoxLayout;
  score_layout->addWidget(new QLabel("Score:"));
  score = QString("NA");
  score_qlabel = new QLabel(score);
  score_layout->addWidget(score_qlabel);

  // Create panel.
  status_widget = new QWidget;

  // Arrange layout in panel
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(team_name_layout);
  layout->addLayout(current_time_layout);
  layout->addLayout(reports_left_layout);
  layout->addLayout(score_layout);
  setLayout( layout );

  status_widget->setEnabled( false );

  status_service_client = nh.serviceClient<interface_protocol::GetStatus>("get_status");

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(getStatus()));
  timer->start(1000);
}

void StatusPanel::getStatus() {
  interface_protocol::GetStatusRequest get_status_request;
  interface_protocol::GetStatusResponse get_status_response;
  status_service_client.call(get_status_request, get_status_response);
  team_name_qlabel->setText(QString::fromStdString(get_status_response.current_team.c_str()));
  current_time_qlabel->setText(QString::fromStdString(std::to_string(get_status_response.run_clock)));
  reports_left_qlabel->setText(QString::fromStdString(std::to_string(get_status_response.remaining_reports)));
  score_qlabel->setText(QString::fromStdString(std::to_string(get_status_response.score)));
}

void StatusPanel::save(rviz::Config config) const
{
  rviz::Panel::save( config );
}

void StatusPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darpa_dashboard::StatusPanel,rviz::Panel )
// END_TUTORIAL
