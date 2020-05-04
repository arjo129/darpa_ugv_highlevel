#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <amapper/rviz/ElevationDisplay.h>
#include <amapper/rviz/ElevationVisual.h>

namespace amapper_elevation_rviz
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ElevationDisplay::ElevationDisplay()
{
  ROS_INFO("Instantiated elevation plugin");
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void ElevationDisplay::onInitialize()
{
  MFDClass::onInitialize();
  visuals_.rset_capacity(10);
  //updateHistoryLength();
}

ElevationDisplay::~ElevationDisplay()
{
}

// Clear the visuals by deleting their objects.
void ElevationDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// This is our callback to handle an incoming message.
void ElevationDisplay::processMessage( const amapper::ElevationGridMsg::ConstPtr& msg )
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<ElevationVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new ElevationVisual( context_->getSceneManager(), scene_node_ ));
  }

  // Now set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(amapper_elevation_rviz::ElevationDisplay, rviz::Display)