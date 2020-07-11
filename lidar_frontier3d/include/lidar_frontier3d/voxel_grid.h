#ifndef _VOXEL_GRID_H_
#define _VOXEL_GRID_H_

#include <pcl/point_types.h>
#include <unordered_set>
#include <tuple>



template<unsigned int res>
struct VoxelIndex {
    std::tuple<long, long, long> index;
    VoxelIndex(pcl::PointXYZ pt) {
        long x = round(pt.x*res);
        long y = round(pt.y*res);
        long z = round(pt.z*res);
        index = std::tuple<long,long,long>(x,y,z);
    }

    VoxelIndex(long x, long y, long z) {
        index = std::tuple<long,long,long>(x,y,z);
    }

    bool operator== (const VoxelIndex& other) const {
        return index == other.index;
    }

    pcl::PointXYZ getPoint() {
        return pcl::PointXYZ(std::get<0>(index), std::get<1>(index), std::get<2>(index));
    }
};

/**
 * res describes a fraction of 1 meter to represent in the grid. 4 wopuld mean each voxel gird is 25cm in size
 */ 
template<unsigned int res>
struct PointHash {
    size_t operator() (VoxelIndex<res> ind) const {
        return std::hash<std::tuple<long, long, long>>()(ind.index) ;
    }
};

/**
 * Perform resenham
 */ 
template<unsigned int res>
void bresenham3D(VoxelIndex<res> start, VoxelIndex<res> end,  std::unordered_set<VoxelIndex<res>, PointHash<res>>& output){
    
    long x1 = std::get<0>(start.index);
    long y1 = std::get<1>(start.index);
    long z1 = std::get<2>(start.index);

    long x2 = std::get<0>(end.index);
    long y2 = std::get<1>(end.index);
    long z2 = std::get<2>(end.index);

    long i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
    long point[3];
    
    point[0] = x1;
    point[1] = y1;
    point[2] = z1;
    dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;
    x_inc = (dx < 0) ? -1 : 1;
    l = abs(dx);
    y_inc = (dy < 0) ? -1 : 1;
    m = abs(dy);
    z_inc = (dz < 0) ? -1 : 1;
    n = abs(dz);
    dx2 = l << 1;
    dy2 = m << 1;
    dz2 = n << 1;
    
    if ((l >= m) && (l >= n)) {
        err_1 = dy2 - l;
        err_2 = dz2 - l;
        for (i = 0; i < l; i++) {
            output.erase(VoxelIndex<res>(point[0], point[1], point[2]));
            if (err_1 > 0) {
                point[1] += y_inc;
                err_1 -= dx2;
            }
            if (err_2 > 0) {
                point[2] += z_inc;
                err_2 -= dx2;
            }
            err_1 += dy2;
            err_2 += dz2;
            point[0] += x_inc;
        }
    } else if ((m >= l) && (m >= n)) {
        err_1 = dx2 - m;
        err_2 = dz2 - m;
        for (i = 0; i < m; i++) {
            output.erase(VoxelIndex<res>(point[0], point[1], point[2]));
            if (err_1 > 0) {
                point[0] += x_inc;
                err_1 -= dy2;
            }
            if (err_2 > 0) {
                point[2] += z_inc;
                err_2 -= dy2;
            }
            err_1 += dx2;
            err_2 += dz2;
            point[1] += y_inc;
        }
    } else {
        err_1 = dy2 - n;
        err_2 = dx2 - n;
        for (i = 0; i < n; i++) {
            output.erase(VoxelIndex<res>(point[0], point[1], point[2]));
            if (err_1 > 0) {
                point[1] += y_inc;
                err_1 -= dz2;
            }
            if (err_2 > 0) {
                point[0] += x_inc;
                err_2 -= dz2;
            }
            err_1 += dy2;
            err_2 += dx2;
            point[2] += z_inc;
        }
    }
    output.erase(VoxelIndex<res>(point[0], point[1], point[2]));
}

template<unsigned int res>
class VoxelGrid {
private:
    std::unordered_set<VoxelIndex<res>, PointHash<res>> visited;
public:
    void addPoint(pcl::PointXYZ pt) {
        visited.insert(VoxelIndex<res>(pt));
    }

    void clearRay(pcl::PointXYZ start, pcl::PointXYZ end) {
        VoxelIndex<res> _start(start.x, start.y, start.z);
        VoxelIndex<res> _end(end.x, end.y, end.z);
        bresenham3D<res>(start, end, visited);
    }

};


#endif