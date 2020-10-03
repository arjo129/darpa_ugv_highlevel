#ifndef _AMAPPER_GRID_
#define _AMAPPER_GRID_

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>

namespace AMapper {

    /**
     * Grid class: Essentially stores a grid. 
     * Has an implementation of bressenham for fast ray tracing. 
     * Follows ROS ENU Convention.
     */ 
    class Grid {
    private:
        float xAnchor, yAnchor; ///Units [m]
        float resolution; ///Units [m/cell]
        std::string frameId;
    public:
        
        /**
         * Size of the grid. Units in cells.
         * Width is in x direction (east), height is in y direction (north).
         */ 
        int16_t gridWidth, gridHeight;

        /**
         * Size of the grid. Units in meters.
         */ 
        double gridMetricWidth, gridMetricHeight;
        /**
         * Raw data per grid. Making it public allows other 
         * functions to manipulate the data. Note the access is
         * data[y][x].
         */ 
        int16_t** data;

        Grid();
        Grid(float xAnchor, float yAnchor, int16_t gridWidth, int16_t gridHeight, float resolution);
        Grid(const Grid& grid);
        Grid(nav_msgs::OccupancyGrid occupancyGrid);
        ~Grid();

        /**
         * Set the tf frame in which grid is in
         */ 
        void setFrameId(std::string frame_id);
        /**
         * Set the tf frame in which grid is in
         */
        std::string getFrameId();
        
        /**
         * Get the resolution of the grid
         */
        float getResolution(); 
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
            return isWithin(x, xAnchor - gridMetricWidth / 2.0, xAnchor + gridMetricWidth / 2.0) && 
                isWithin(y, yAnchor - gridMetricHeight / 2.0, yAnchor + gridMetricHeight / 2.0);
        }
        
        /**
         * 
         */ 
        inline bool isWithinGridCellMap(int x_idx, int y_idx) {
            return isWithin(x_idx, 0, gridWidth - 1) && 
                   isWithin(y_idx, 0, gridHeight - 1);
        }

        /**
         * Converts world coordinmate in y axis to map y axis
         */ 
        inline int toYIndex(float yValue) {
            return this->gridHeight/2 + (int)round((yValue-yAnchor)/resolution);
        }

        inline int toXIndex(float xValue) {
            return this->gridWidth/2 + (int)round((xValue-xAnchor)/resolution);
        }

        inline float fromXIndex(int xValue) {
            return (xValue-this->gridWidth/2)*resolution + xAnchor;
        }

        inline float fromYIndex(int yValue) {
            return (yValue-this->gridHeight/2)*resolution + yAnchor;
        }

        void clear();

        bool operator ==(const Grid &b) const;
    };

    /**
     * Inflates an occupancy grid
     */ 
    Grid* inflate(Grid& grid, int radius);

    
};

#endif