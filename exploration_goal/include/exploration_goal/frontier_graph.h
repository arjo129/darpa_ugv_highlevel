// Data structure to manage the graphs and related helper functions

#ifndef FRONTIER_GRAPH_H_
#define FRONTIER_GRAPH_H_

#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>

#include <nanoflann/nanoflann.hpp>

struct PointCloud3d_ {
    std::vector<Eigen::Vector3f> pts;
    
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
		L2_Simple_Adaptor<float, PointCloud3d_ > ,
		PointCloud3d_,
		3
		> kd_tree_t;



class FrontierGraph 
{
    public:
        FrontierGraph();
        FrontierGraph(graph_msgs::GeometryGraph graph);
        ~FrontierGraph();
        FrontierGraph mergeLocalGraph (const FrontierGraph & local_graph);
        graph_msgs::GeometryGraph toMsg();
        void markAsVisited(int node_idx);

    private:
        vector<int> visited;
        vector<set<int>> adjacency_list; //maybe can use map for O(1) ?
        int num_nodes;
};







#endif /* FRONTIER_GRAPH_H_ */