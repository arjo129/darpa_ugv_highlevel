// Data structure to manage the graphs and related helper functions

#ifndef FRONTIER_GRAPH_H_
#define FRONTIER_GRAPH_H_

#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>
#include <std_msgs/UInt32.h>
#include <Eigen/Dense>

#include <nanoflann/nanoflann.hpp>
#include <vector>
#include <algorithm>

// struct PointCloud3d_ {
//     std::vector<Eigen::Vector3f> pts;
    
//     inline float kdtree_get_pt(const size_t idx, const size_t dim) const
// 	{
// 		if (dim == 0) return pts[idx].x;
// 		else if (dim == 1) return pts[idx].y;
// 		else return pts[idx].z;
// 	}

//     inline size_t kdtree_get_point_count() const { return pts.size(); }

//     template <class BBOX>
// 	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

// };

// typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
// 		L2_Simple_Adaptor<float, PointCloud3d_ > ,
// 		PointCloud3d_,
// 		3
// 		> kd_tree_t;

typedef enum GraphNodeExploredState 
{
    NODE_UNEXPLORED = 0,
    NODE_EXPLORING = 1,
    NODE_EXPLORED = 2
} GraphNodeExploredState;

class FrontierGraph 
{
    public:
        FrontierGraph();
        FrontierGraph(graph_msgs::GeometryGraph graph);
        ~FrontierGraph();
        FrontierGraph mergeLocalGraph (const FrontierGraph & local_graph);
        graph_msgs::GeometryGraph toMsg();
        void markAsUnExplored(int node_idx);
        void markAsExploring(int node_idx);
        void markAsExplored(int node_idx);

    private:
        std::vector<int> explored_state;
        std::vector<std::set<int>> adjacency_list;
        std::vector<geometry_msgs::Point> node_idx_to_3d_point;
        int num_nodes;
};


FrontierGraph::FrontierGraph() : num_nodes(0)
{}

FrontierGraph::FrontierGraph(graph_msgs::GeometryGraph graph) : num_nodes(graph.nodes.size()),
                                                                explored_state(num_nodes, NODE_UNEXPLORED),
                                                                adjacency_list(num_nodes, std::set<int>())
{
    // 0-based List containing mapping of node index to 3D points.
    node_idx_to_3d_point = graph.nodes; 

    // Convert vector based adj_list to set based adj_list for easier insertion when merging with other graphs
    for (int idx=0; idx<num_nodes; idx++)
    {
        std::vector<int> adj_vec;
        for (auto edge : graph.edges[idx].node_ids)
        {
            adj_vec.push_back(edge);
        }
        std::copy(adj_vec.begin(), adj_vec.end(), std::inserter(adjacency_list[idx], adjacency_list[idx].end()));
    }
}

FrontierGraph::~FrontierGraph()
{}

FrontierGraph FrontierGraph::mergeLocalGraph (const FrontierGraph & local_graph)
{

}

graph_msgs::GeometryGraph FrontierGraph::toMsg()
{
    graph_msgs::GeometryGraph graph;

    graph.header.frame_id = "world";
    graph.header.stamp = ros::Time::now();

    graph.nodes = node_idx_to_3d_point;
    
    for (int idx=0; idx<num_nodes; idx++)
    {
        std::vector<int> adj_vec(adjacency_list[idx].begin(), adjacency_list[idx].end());
        graph.edges.push_back(graph_msgs::Edges());
        for (auto edge : adj_vec)
        {
            graph.edges[idx].node_ids.push_back(edge);
        }
    }

    return graph;
}

void FrontierGraph::markAsUnExplored(int node_idx) {explored_state[node_idx] = NODE_UNEXPLORED;}
void FrontierGraph::markAsExploring(int node_idx) { explored_state[node_idx] = NODE_EXPLORING; }
void FrontierGraph::markAsExplored(int node_idx) { explored_state[node_idx] = NODE_EXPLORED; }



#endif /* FRONTIER_GRAPH_H_ */