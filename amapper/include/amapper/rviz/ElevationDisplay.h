
#ifndef ELEV_DISPLAY_H
#define ELEV_DISPLAY_H

#include <boost/circular_buffer.hpp>
#include <amapper/rviz/ElevationVisual.h>
#include <amapper/ElevationGridMsg.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace amapper_elevation_rviz
{
class ElevationDisplay: public rviz::MessageFilterDisplay<amapper::ElevationGridMsg>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ElevationDisplay();
  virtual ~ElevationDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();
  // Function to handle an incoming ROS message.
private Q_SLOTS:
private:
  void processMessage( const amapper::ElevationGridMsg::ConstPtr& msg );

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<ElevationVisual> > visuals_;
};
};
#endif