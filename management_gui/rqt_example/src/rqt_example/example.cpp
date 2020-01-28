#include "rqt_example/example.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_example {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

Example::Example()
    : rqt_gui_cpp::Plugin(),
      widget_(nullptr) {

  setObjectName("Example");
  model = new QStringListModel();
  vehicleList << "UGV 1" << "CrazyFlie"  << "TGV";
  model->setStringList(vehicleList);
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void Example::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);
  ui_.optionList->addItem("Show All Robots", "robotView");
  ui_.optionList->addItem("Show All Artifacts", "artifactView");
  ui_.listView->setModel(model);
  artifactStream = nh_.subscribe("/artifact/", 10, &Example::onArtifactDetected, this);
  vehicleList << "UGV 1";
}

void Example::shutdownPlugin() {
}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void Example::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const {
}

void Example::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings) {
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */



/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */



/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */
void Example::onArtifactDetected(std_msgs::String msg) {
  artifactList << QString::fromStdString(msg.data);
  
}


/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */



/* ========================================================================== */
/* Signals                                                                    */
/* ========================================================================== */



} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_example::Example,
                       rqt_gui_cpp::Plugin)
