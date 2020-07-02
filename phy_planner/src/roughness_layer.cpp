/**
 * Roughness layer uses realsense normal etimate's point cloud to define the difficulty of each point on the costmap
 * (c) Arjo Chakravarty 2020
 */ 

#include <phy_planner/roughness_layer.h>
#include <phy_planner/elevation_profiler.h>
#include <pluginlib/class_list_macros.h>

#define radius 5
#define increment 0.05
PLUGINLIB_EXPORT_CLASS(phy_planner::RoughnessLayer, costmap_2d::Layer)


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
  
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "");
  if(topic_name == ""){
    ROS_ERROR("No topic name given");
    return;
  }

  std::string world_frame;
  nh.param<std::string>("world_frame", world_frame, "");
  if(topic_name == world_frame){
    ROS_ERROR("No world frame name given");
    return;
  }

  std::string robot_frame;
  nh.param<std::string>("robot_frame", robot_frame, "");
  if(topic_name == robot_frame){
    ROS_ERROR("No robot frame name given");
    return;
  }

  double robot_height;
  if(nh.param<double>("robot_height", robot_height, 0.3)){
    ROS_WARN("No robot height given. Assuming 30cm height");
    robot_height = 0.3;
  }

  terrain_profile = boost::make_shared<ElevationProfiler>(new ElevationProfiler(topic_name, world_frame, robot_frame, robot_height));
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
  if (!enabled_)
    return;
  
  boost::shared_ptr<AMapper::ElevationGrid> elevation_grid = this->terrain_profile->final_map;

  auto res = elevation_grid->getResolution();
  if(res == 0) return;
  
  for(double i = -radius; i <  radius; i+=res){
    for(double j = -radius; j <  radius; j+=res){
      auto center_x = robot_x_+i;
      auto center_y = robot_y_+j;
      //ROS_INFO("%f %f", i,j);
      auto elevation = elevation_grid->queryElevation(center_x, center_y);
      unsigned int mx, my;
      auto gotTransform = master_grid.worldToMap(center_x, center_y, mx, my);
        
      if(!gotTransform) {
        continue;
      }

      if(elevation == -INFINITY) {
        master_grid.setCost(mx, my, costmap_2d::NO_INFORMATION);
        continue;
      }

      auto elevation_dx = elevation_grid->queryElevation(center_x-res, center_y);
      if(elevation_dx == -INFINITY) {;
          master_grid.setCost(mx, my, costmap_2d::NO_INFORMATION);
          continue;
      }
      
      elevation_dx = (elevation - elevation_dx)/ res;
      if(abs(elevation_dx) > 1) {
          master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
          continue;
      }

      auto elevation_dy = elevation_grid->queryElevation(center_x, center_y-res);
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


}