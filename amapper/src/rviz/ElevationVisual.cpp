#include <ros/ros.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/point_cloud.h>

#include <amapper/rviz/ElevationVisual.h>
#include <amapper/elevation_grid.h>

namespace amapper_elevation_rviz
{

// BEGIN_TUTORIAL
ElevationVisual::ElevationVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relatArrowive to its header frame.
  point_cloud_.reset(new rviz::PointCloud());
  point_cloud_->setName("sname.str()");
  point_cloud_->setRenderMode(rviz::PointCloud::RM_BOXES);
  point_cloud_->setVisible(true);
  point_cloud_->setDimensions(0.02,0.02,0.02);
  frame_node_->attachObject(point_cloud_.get());

}

ElevationVisual::~ElevationVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void ElevationVisual::setMessage( const amapper::ElevationGridMsg::ConstPtr& msg )
{
  ///auto grid = boost::make_shared<AMapper::ElevationGrid>(*msg);
  point_cloud_->clear();
  std::vector<rviz::PointCloud::Point> points;
  for(auto data: msg->data) {
    if(fpclassify(data.elevation) != FP_NORMAL)
      continue;
    rviz::PointCloud::Point pt;
    pt.position.x = data.x;
    pt.position.y = data.y;
    pt.position.z = data.elevation;
    pt.setColor(1.0, 0, 1.0, 0.5);
    points.push_back(pt);
  }
  point_cloud_->addPoints(points.data(), points.size());
}

// Position and orientation are passed through to the SceneNode.
void ElevationVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void ElevationVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void ElevationVisual::setColor( float r, float g, float b, float a )
{
 // point_cloud_->setColor( r, g, b, a );
}
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials
