#ifndef _AMAPPER_NO_CLEAR_RAYTRACER_
#define _AMAPPER_NO_CLEAR_RAYTRACER_
#include <amapper/RayTracer.h>

namespace AMapper {
    class NoClearingRaytracer : public RayTracer {
        public:
        NoClearingRaytracer() {};
        private:
        void plotFreeSpace(Grid& grid, Eigen::Vector2i centroid, int x, int y) override;
    };
}
#endif