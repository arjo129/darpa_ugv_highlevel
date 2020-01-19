#include <amapper/cluster.h>
#include <unordered_set>
#include <queue>
namespace std
{
  template<>
    struct hash<Eigen::Vector2i>
    {
      size_t
      operator()(const Eigen::Vector2i & obj) const
      {
        return hash<int>()(obj.y()) ^ hash<int>()(obj.x());
      }
    };
}
namespace AMapper {

Cluster expandCluster(Grid& grid, std::unordered_set<Eigen::Vector2i>& explored, int x, int y) {
    Eigen::Vector2i start(x,y);
    std::queue<Eigen::Vector2i> queue;
    std::vector<Eigen::Vector2i> directions;
    Cluster cluster;
    queue.push(Eigen::Vector2i(x,y));
    explored.insert(start);
    cluster.push_back(start);

    for(int i = -1; i < 2; i++){
        for(int j = -1; j < 2; j++){
            directions.push_back(Eigen::Vector2i(i,j));
        }
    }
    
    while(!queue.empty()){
        Eigen::Vector2i point = queue.front();
        queue.pop();
        for(Eigen::Vector2i direction: directions){
            Eigen::Vector2i pt = point + direction;
            if(!grid.isWithinGridCellMap(pt.x(), pt.y())) continue;
            if(explored.count(pt) != 0) continue;
            explored.insert(pt);
            if(grid.data[pt.y()][pt.x()]==100){
                queue.push(pt);
                cluster.push_back(pt);
            }
        }
    }
    return cluster;
}

std::vector<Cluster> getClusters(Grid& grid) {
    std::unordered_set<Eigen::Vector2i> explored;
    std::vector<Cluster> clusters;
    for(int y = 0; y < grid.gridWidth; y++){
        for(int x = 0; x < grid.gridHeight; x++){
            if(grid.data[y][x] == 100 && explored.count(Eigen::Vector2i(x,y)) == 0){
                Cluster cluster = expandCluster(grid, explored, x, y);
                clusters.push_back(cluster);
            }
        }
    }
    return clusters;
}

Eigen::Vector2i getCentroid(Cluster cluster){
    Eigen::Vector2i centroid(0 ,0);
    for(int i = 0; i < cluster.size(); i++){
        centroid += cluster[i];
    }
    return centroid/cluster.size();
}
}