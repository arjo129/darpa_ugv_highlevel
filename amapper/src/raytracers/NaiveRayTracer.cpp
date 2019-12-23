#include <amapper/raytracers/NaiveRaytracer.h>

using namespace AMapper;

void NaiveRaytracer::plotFreeSpace(Grid& grid, Eigen::Vector2i centroid, int x, int y) {
    grid.data[y][x] = 0;
}