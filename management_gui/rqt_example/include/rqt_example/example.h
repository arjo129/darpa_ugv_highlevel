#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <rqt_example/ui_example.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QWidget>
#include <QObject>
#include <QStringList>
#include <QStringListModel>

namespace rqt_example {

class Example : public rqt_gui_cpp::Plugin {
Q_OBJECT

public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  Example();

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void shutdownPlugin() override;

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                    qt_gui_cpp::Settings& instance_settings) const override;

  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                       const qt_gui_cpp::Settings& instance_settings) override;

private:

  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const std::string TAG = "Example";

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::ExampleWidget ui_;
  QWidget* widget_;

  ros::NodeHandle nh_;
  ros::Subscriber artifactStream;
  QStringListModel* model;
  QStringList vehicleList;
  QStringList artifactList;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  

  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */



  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */
  void onArtifactDetected(std_msgs::String msg);


protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */



signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */



};

} // namespace
