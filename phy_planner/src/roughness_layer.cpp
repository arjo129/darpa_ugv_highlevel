/**
 * Roughness layer uses realsense normal etimate's point cloud to define the difficulty of each point on the costmap
 * (c) Arjo Chakravarty 2020
 */ 

#include <phy_planner/roughness_layer.h>
#include <pluginlib/class_list_macros.h>

#define radius 5
#define increment 0.05
PLUGINLIB_EXPORT_CLASS(phy_planner::RoughnessLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace phy_planner
{

RoughnessLayer::RoughnessLayer() {}

void RoughnessLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &RoughnessLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  sub = nh.subscribe("/plane_segmentation/grid_map", 10, &RoughnessLayer::onElevMsg, this);
}


void RoughnessLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void RoughnessLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  ROS_INFO("Got new bounds");
  if (!enabled_)
    return;

  robot_x_ = robot_x;
  robot_y_ = robot_y;

  *min_x = robot_x_ - radius;
  *min_y = robot_y_ - radius;
  *max_x = robot_x_ + radius;
  *max_y = robot_y_ + radius;
}

void RoughnessLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  ROS_INFO("updating costs");
  if (!enabled_)
    return;
  
  AMapper::ElevationGrid elevation_grid(this->grid);
  auto res = elevation_grid.getResolution();
  ROS_INFO("Querying %f %f", robot_x_, robot_y_);
  if(res == 0) return;
  //res =0.1;
  for(double i = -radius; i <  radius; i+=res){
    for(double j = -radius; j <  radius; j+=res){
      auto center_x = robot_x_+i;
      auto center_y = robot_y_+j;
      //ROS_INFO("%f %f", i,j);
      auto elevation = elevation_grid.queryElevation(center_x, center_y);
      unsigned int mx, my;
      auto gotTransform = master_grid.worldToMap(center_x, center_y, mx, my);
        
      if(!gotTransform) {
        continue;
      }
      //master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
      if(elevation == -INFINITY) {
        master_grid.setCost(mx, my, costmap_2d::NO_INFORMATION);
        continue;
      }

      auto elevation_dx = elevation_grid.queryElevation(center_x-res, center_y);
      if(elevation_dx == -INFINITY) {;
          master_grid.setCost(mx, my, costmap_2d::NO_INFORMATION);
          continue;
      }
      
      elevation_dx = (elevation - elevation_dx)/ res;
      if(abs(elevation_dx) > 1) {
          master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
          continue;
      }

      auto elevation_dy = elevation_grid.queryElevation(center_x, center_y-res);
      if(elevation_dy == -INFINITY) {
          master_grid.setCost(mx, my, costmap_2d::NO_INFORMATION);
          continue;
      }

      elevation_dy = (elevation - elevation_dy)/ res;
      //ROS_INFO("%f gradient", elevation_dy);
      if(abs(elevation_dy) > 1) {
          master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
          continue;
      }

      auto cost = elevation_dx*elevation_dx + elevation_dy*elevation_dy;
      cost*=230;
      master_grid.setCost(mx, my, cost);
    }
  }
    
}

void RoughnessLayer::onElevMsg(amapper::ElevationGridMsg msg) {
    this->grid = msg;
}

}