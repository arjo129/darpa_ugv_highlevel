#include <amapper/raytracers/NoClearingRaytracer.h>

using namespace AMapper;

void NoClearingRaytracer::plotFreeSpace(Grid& grid, Eigen::Vector2i centroid, int x, int y) {
    if(grid.data[y][x] < 0)
        grid.data[y][x] = 0;
}