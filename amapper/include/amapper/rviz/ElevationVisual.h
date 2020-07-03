
#ifndef ELEV_VISUAL_H
#define ELEV_VISUAL_H

#include <amapper/ElevationGridMsg.h>
#include <rviz/ogre_helpers/point_cloud.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class SceneNode;
class SceneManager;
}

namespace rviz
{
class Arrow;
}

namespace amapper_elevation_rviz
{

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of ImuVisual represents the visualization of a single
// sensor_msgs::Imu message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class ElevationVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  ElevationVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~ElevationVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const amapper::ElevationGridMsg::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way ImuVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Imu message.
  void setColor( float r, float g, float b, float a );

private:
  // The object implementing the actual arrow shape
  boost::shared_ptr<rviz::PointCloud> point_cloud_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Imu message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};

} ;

#endif 