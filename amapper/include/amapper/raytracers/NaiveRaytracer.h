#ifndef _AMAPPER_NAIVE_RAYTRACER_
#define _AMAPPER_NAIVE_RAYTRACER_
#include <amapper/RayTracer.h>

namespace AMapper {
    class NaiveRaytracer : public RayTracer {
        public:
        NaiveRaytracer() {};
        private:
        /**
         * @param occupancy grid
         * @param centroid Current robot position
         * @param X, Y - Target coordinates
         */ 
        void plotFreeSpace(Grid& grid, Eigen::Vector2i centroid, int x, int y) override;
    };
}
#endif