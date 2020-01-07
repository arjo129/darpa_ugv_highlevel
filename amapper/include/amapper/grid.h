#ifndef _AMAPPER_GRID_
#define _AMAPPER_GRID_

#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace AMapper {

    /**
     * Grid class: Essentially stores a grid. 
     * Has an implementation of bressenham for fast ray tracing
     */ 
    class Grid {
    private:
        float xAnchor, yAnchor; ///Units [m]
        float resolution; ///Units [m/cell]
        std::string frameId;
    public:
        
        /**
         * Size of the grid. Units in cells.
         */ 
        int gridSize;

        /**
         * Size of the grid. Units in meters.
         */ 
        double gridMetricSize;
        /**
         * Raw data per grid. Making it public allows other 
         * functions to manipulate the data. Note the access is
         * data[y][x].
         */ 
        int16_t** data;

        Grid();
        Grid(float xAnchor, float yAnchor, int gridSize, float resolution);
        Grid(Grid& grid);
        Grid(nav_msgs::OccupancyGrid occupancyGrid);

        /**
         * Set the tf frame in which grid is in
         */ 
        void setFrameId(std::string frame_id);
        
        /**
         * Convert to occupancy grid
         */ 
        nav_msgs::OccupancyGrid toOccupancyGrid();


        /**
         * Some helper functions to help with grid indexing
         */
        inline bool isWithin(double val_to_test, double min_val, double max_val) {
            return val_to_test >= min_val && val_to_test <= max_val;
        }

        inline bool isWithinMetricMap(double x, double y) {
            return isWithin(x, xAnchor - gridMetricSize / 2.0, xAnchor + gridMetricSize / 2.0) && 
                isWithin(y, yAnchor - gridMetricSize / 2.0, yAnchor + gridMetricSize / 2.0);
        }

        inline bool isWithinGridCellMap(int x_idx, int y_idx) {
            return isWithin(x_idx, 0, gridSize - 1) && 
                   isWithin(y_idx, 0, gridSize - 1);
        }

        inline int toYIndex(float yValue) {
            return this->gridSize/2 + (int)round((yValue-yAnchor)/resolution);
        }

        inline int toXIndex(float xValue) {
            return this->gridSize/2 + (int)round((xValue-xAnchor)/resolution);
        }

        inline float fromXIndex(int xValue) {
            return (xValue-this->gridSize/2)*resolution + xAnchor;
        }

        inline float fromYIndex(int yValue) {
            return (yValue-this->gridSize/2)*resolution + yAnchor;
        }

        void clear();

        bool operator ==(const Grid &b) const;
    };

};

#endif