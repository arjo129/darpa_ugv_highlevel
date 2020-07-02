#ifndef ROUGHNESS_LAYER_H
#define ROUGHNESS_LAYER_H
#include <ros/ros.h>
#include <amapper/ElevationGridMsg.h>
#include <amapper/elevation_grid.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "elevation_profiler.h"

namespace phy_planner {
    class RoughnessLayer: public costmap_2d::CostmapLayer {
    public:
        RoughnessLayer();
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
        void onElevMsg(amapper::ElevationGridMsg msg);
        double robot_x_, robot_y_;
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
        ros::Subscriber sub;
        amapper::ElevationGridMsg grid;
        boost::shared_ptr<ElevationProfiler> terrain_profile;
    };
}
#endif