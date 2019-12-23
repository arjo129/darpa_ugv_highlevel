#ifndef _AMAPPER_RayTracer_
#define _AMAPPER_RayTracer_
#include <Eigen/Eigen>
#include "grid.h"

namespace AMapper {
class RayTracer {
    private:
         virtual void plotFreeSpace(Grid& grid, Eigen::Vector2i centroid, int x, int y) = 0;
    public:
       
         /**
         * Implements the bresenham algorithm. 
         */ 
        void rayTrace(Grid& grid, Eigen::Vector2f center, Eigen::Vector2f end);

};
}
#endif