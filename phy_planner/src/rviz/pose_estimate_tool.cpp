#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <phy_planner/final_pose_estimate.h>
#include "pose_estimate_tool.h"

namespace phy_planner
{

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
PoseEstimateTool::PoseEstimateTool()
{
  shortcut_key_ = 'e';
  shape = NULL;
  ros::NodeHandle n;
  client = n.serviceClient<phy_planner::final_pose_estimate>("pose_estimate");
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
PoseEstimateTool::~PoseEstimateTool()
{
  
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void PoseEstimateTool::onInitialize()
{
  robotScene_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  robotScene_->setVisible(false);
  shape = new rviz::Shape(rviz::Shape::Cube, scene_manager_, robotScene_);
  shape->setColor(1,0,0,1);
}

void PoseEstimateTool::activate()
{

}


void PoseEstimateTool::deactivate()
{
  
}


int PoseEstimateTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  robotScene_->setVisible(false);
  if(!client.exists()) return Render;
  if(robotScene_ == NULL) return Render;
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    phy_planner::final_pose_estimate srv;
    srv.request.target_pose.position.x = intersection.x;
    srv.request.target_pose.position.y = intersection.y;
    srv.request.target_pose.position.z = intersection.z;

    srv.request.target_pose.orientation.x = 0;
    srv.request.target_pose.orientation.y = 0;
    srv.request.target_pose.orientation.z = 0;
    srv.request.target_pose.orientation.w = 1;

    auto res = client.call(srv);

    if(!res) {
      ROS_WARN("Server likely crashed");
      return Render;
    }
    if(srv.response.ok) {
      intersection.z = srv.response.final_pose.position.z;
      shape->setColor(0,1,0,1);
      Ogre::Quaternion finalQuaternion;
      finalQuaternion.x = srv.response.final_pose.orientation.x;
      finalQuaternion.y = srv.response.final_pose.orientation.y;
      finalQuaternion.z = srv.response.final_pose.orientation.z;
      finalQuaternion.w = srv.response.final_pose.orientation.w;
      shape->setOrientation(finalQuaternion);
    }
    else {
      ROS_WARN("Robot fell");
      ROS_INFO("Position: %f %f %f", 
          srv.response.final_pose.position.x,
          srv.response.final_pose.position.y,
          srv.response.final_pose.position.z);
      ROS_INFO("Orientation: %f %f %f %f",
          srv.response.final_pose.orientation.x,
          srv.response.final_pose.orientation.y,
          srv.response.final_pose.orientation.z,
          srv.response.final_pose.orientation.w
      );
      shape->setColor(1,0,0,1);
    }
    ROS_INFO("Intersect at: %f %f %f", intersection.x, intersection.y, intersection.z);
    shape->setPosition(intersection);
    robotScene_->setVisible(true);
  }
  return Render;
}


// Loading and saving the flags
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named flag positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void PoseEstimateTool::save( rviz::Config config ) const
{
  
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void PoseEstimateTool::load( const rviz::Config& config )
{
  
}

} // end namespace ph_planner
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(phy_planner::PoseEstimateTool, rviz::Tool)