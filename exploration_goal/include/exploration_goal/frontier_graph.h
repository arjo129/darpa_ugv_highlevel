// Data structure to manage the graphs and related helper functions

#ifndef FRONTIER_GRAPH_H_
#define FRONTIER_GRAPH_H_

#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>

#include <nanoflann/nanoflann.hpp>
#include <vector>
#include <algorithm>

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
        geometry_msgs::Point getPointForNodeId(int node_idx);
        int getSize();
        graph_msgs::GeometryGraph toMsg();
        void markAsUnExplored(int node_idx);
        void markAsExploring(int node_idx);
        void markAsExplored(int node_idx);

    private:
        std::vector<int> explored_state;
        std::vector<std::set<int>> adjacency_list;
        std::vector<geometry_msgs::Point> node_idx_to_3d_point;
        int num_nodes;
        int current_node_idx;
};


FrontierGraph::FrontierGraph() : num_nodes(0)
{}

FrontierGraph::FrontierGraph(graph_msgs::GeometryGraph graph) : num_nodes(graph.nodes.size()),
                                                                explored_state(num_nodes, NODE_UNEXPLORED),
                                                                adjacency_list(num_nodes, std::set<int>(0)),
                                                                current_node_idx{0}
{
    // 0-based List containing mapping of node index to 3D points.
    node_idx_to_3d_point = graph.nodes; 

    // Convert vector based adj_list to set based adj_list for easier insertion when merging with other graphs
    for (int idx=0; idx<num_nodes; idx++)
    {
        vector<int> adj_vec = graph.edges[idx];
        std::copy(adj_vec.begin(), adj_vec.end(), std::inserter(adjacency_list[idx], adjacency_list[idx].end()));
    }
}

FrontierGraph::~FrontierGraph()
{}

geometry_msgs::Point FrontierGraph::getPointForNodeId(int node_idx){
    return node_idx_to_3d_point[node_idx];
}

int FrontierGraph::getSize(){
    return num_nodes;
}

FrontierGraph::FrontierGraph mergeLocalGraph (const FrontierGraph & local_graph)
{
    for(int i = 0 ; local_graph.size() ; i ++){
        for(auto &b: local_graph[i]){
            if(i == 0){
                adjacency_list[current_node_idx].insert(b+num_nodes-1);
            }else{
                std::set<int>temp_set;
                temp_set.insert(b+num_nodes-1);
                adjacency_list.push_back(temp_set);
                explored_state.push_back(GraphNodeExploredState::NODE_UNEXPLORED);
                node_idx_to_3d_point.push_back(local_graph.getPointForNodeId(i));
            }
        }
    }
    num_nodes += local_graph.getSize();
}

FrontierGraph::graph_msgs::GeometryGraph toMsg()
{
    graph_msgs::GeometryGraph graph;

    graph.header.frame_id = "world";
    graph.header.stamp = ros::Time::now();

    graph.nodes = node_idx_to_3d_point;
    
    for (int idx=0; idx<num_nodes; idx++)
    {
        vector<int> adj_vec(adjacency_list[idx].begin(), adjacency_list[idx].end());
        graph.edges.push_back(adj_vec);
    }

    return graph;
}

FrontierGraph::void markAsUnExplored(int node_idx) {explored_state[node_idx] = NODE_UNEXPLORED;}
FrontierGraph::void markAsExploring(int node_idx) { explored_state[node_idx] = NODE_EXPLORING; }
FrontierGraph::void markAsExplored(int node_idx) { explored_state[node_idx] = NODE_EXPLORED; }



#endif /* FRONTIER_GRAPH_H_ */