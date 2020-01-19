#ifndef _amapper_cluster_h_
#define _amapper_cluster_h_

#include <vector>
#include <Eigen/Eigen>
#include <amapper/grid.h>

namespace AMapper {
    typedef std::vector<Eigen::Vector2i> Cluster;

    std::vector<Cluster> getClusters(Grid& grid);
    Eigen::Vector2i getCentroid(Cluster cluster);
};

#endif