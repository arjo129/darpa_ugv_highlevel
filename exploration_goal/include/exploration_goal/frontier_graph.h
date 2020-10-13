// Data structure to manage the graphs and related helper functions

#ifndef FRONTIER_GRAPH_H_
#define FRONTIER_GRAPH_H_

#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>

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
    NODE_EXPLORED = 2,
    NODE_UNEXPLORABLE = 3
} GraphNodeExploredState;

class FrontierGraph 
{
    public:
        FrontierGraph(std::string global_frame_id_);
        FrontierGraph(graph_msgs::GeometryGraph graph);
        ~FrontierGraph();
        void mergeLocalGraph (FrontierGraph & local_graph);
        geometry_msgs::Point getPointForNodeId(int node_idx);
        std::vector<std::set<int>> & getAL();
        int getSize();
        graph_msgs::GeometryGraph toMsg();
        void markAsUnExplored(int node_idx);
        void markAsExploring(int node_idx);
        void markAsExplored(int node_idx);
        void markAsUnExplorable(int node_idx);
        int getNextGoalId();
        geometry_msgs::PointStamped getNextGoal();
        int getParent(int);
        bool isLeafNode();
        geometry_msgs::PointStamped reset();
        void updateNewGoalSuccess();
        void updateNewGoalFail();
        void setGraph(FrontierGraph temp);

//    protected:
        std::vector<int> explored_state;
        std::vector<std::set<int>> adjacency_list;
        std::vector<geometry_msgs::Point> node_idx_to_3d_point;
        int num_nodes;
        int current_node_idx;
        int upcoming_node_idx;
        std::string global_frame_id;
};


FrontierGraph::FrontierGraph(std::string global_frame_id_) : num_nodes(0), current_node_idx(0), global_frame_id(global_frame_id_)
{}

FrontierGraph::FrontierGraph(graph_msgs::GeometryGraph graph) : num_nodes(graph.nodes.size()), current_node_idx(0) , upcoming_node_idx(0)
{
    explored_state.resize(num_nodes, NODE_UNEXPLORED);
    adjacency_list.resize(num_nodes, std::set<int>{});

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

geometry_msgs::Point FrontierGraph::getPointForNodeId(int node_idx){
    return node_idx_to_3d_point[node_idx];
}

int FrontierGraph::getSize(){
    return num_nodes;
}

std::vector<std::set<int>>& FrontierGraph::getAL(){
    return adjacency_list;
}

void FrontierGraph::setGraph(FrontierGraph temp)
{
    explored_state = temp.explored_state;
    adjacency_list = temp.getAL();
    node_idx_to_3d_point = temp.node_idx_to_3d_point;
    num_nodes = temp.getSize();
    current_node_idx = temp.current_node_idx;
    upcoming_node_idx = temp.upcoming_node_idx;
    explored_state[current_node_idx] = GraphNodeExploredState::NODE_EXPLORED;
}

void FrontierGraph::mergeLocalGraph (FrontierGraph & local_graph){
    if (num_nodes == 0)
    {
        setGraph(local_graph);
        return;
    }
    adjacency_list[current_node_idx].clear(); //Added to ensure only single path of exploration exist
    auto localAL = local_graph.getAL();
    for(int i = 0 ; i < localAL.size() ; i ++){
        std::set<int>temp_set;
        for(auto &b: localAL[i]){
            if(i == 0){
                std::cout << "Adding " << b+num_nodes-1 << " at idx= " << current_node_idx << std::endl;
                adjacency_list[current_node_idx].insert(b+num_nodes-1);
            }else{
                temp_set.insert(b+num_nodes-1);
            }
        }
        if(i != 0){
            adjacency_list.push_back(temp_set);
            explored_state.push_back(GraphNodeExploredState::NODE_UNEXPLORED);
            node_idx_to_3d_point.push_back(local_graph.getPointForNodeId(i));
        }
    }
    num_nodes += local_graph.getSize() - 1;
}

bool FrontierGraph::isLeafNode(){
    if(adjacency_list[current_node_idx].empty()){
        return true;
    }
    for(auto  &child_node: adjacency_list[current_node_idx]){
         if(explored_state[child_node] == GraphNodeExploredState::NODE_EXPLORED || explored_state[child_node] == GraphNodeExploredState::NODE_UNEXPLORED){
            return false;
        }
    }
    return true;
}

int FrontierGraph::getNextGoalId(){
    std::cout << "next goal id. curr = " << current_node_idx << std::endl;
    if(adjacency_list[current_node_idx].empty()){
        return (current_node_idx);
    }
    
    for(auto &child_node: adjacency_list[current_node_idx]){
        if(explored_state[child_node] == GraphNodeExploredState::NODE_EXPLORED || explored_state[child_node] == GraphNodeExploredState::NODE_UNEXPLORABLE){
            continue;
        }else if(explored_state[child_node] == GraphNodeExploredState::NODE_UNEXPLORED){
            std::cout << "next goal id. " << child_node << std::endl;
            return child_node;
        }
    }

    return getParent(current_node_idx);
}


geometry_msgs::PointStamped FrontierGraph::getNextGoal(){
    geometry_msgs::PointStamped goal;
    goal.header.frame_id = global_frame_id;
    goal.header.stamp = ros::Time::now();
    int id = getNextGoalId();
    upcoming_node_idx = id;
    geometry_msgs::Point goalpoint = getPointForNodeId(id);
    goal.point = goalpoint;
    return goal;
}

geometry_msgs::PointStamped FrontierGraph::reset(){
    geometry_msgs::PointStamped goal;
    goal.header.frame_id = global_frame_id;
    goal.header.stamp = ros::Time::now();
    geometry_msgs::Point goalpoint = getPointForNodeId(current_node_idx);
    goal.point = goalpoint;
    return goal;
}

void FrontierGraph::updateNewGoalSuccess(){
    current_node_idx = upcoming_node_idx;
    markAsExplored(current_node_idx);
}

void FrontierGraph::updateNewGoalFail(){
    upcoming_node_idx = current_node_idx;
    markAsUnExplorable(upcoming_node_idx);
}

int FrontierGraph::getParent(int child_node){
    for(int i = 0; i < adjacency_list.size() ; i ++){
        if(adjacency_list[i].find(child_node) != adjacency_list[i].end()){
            return i;
        }
    }
}

graph_msgs::GeometryGraph FrontierGraph::toMsg()
{
    graph_msgs::GeometryGraph graph;

    graph.header.frame_id = global_frame_id;
    graph.header.stamp = ros::Time::now();

    graph.nodes = node_idx_to_3d_point;
    
    for (int idx=0; idx<num_nodes; idx++)
    {
        std_msgs::UInt8 explored_msg;
        if (idx == upcoming_node_idx)
        {
            graph.explored.push_back(GraphNodeExploredState::NODE_EXPLORING);
        }
        else
        {
            graph.explored.push_back(explored_state[idx]);
        }

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
void FrontierGraph::markAsUnExplorable(int node_idx) { explored_state[node_idx] = NODE_UNEXPLORABLE; }



#endif /* FRONTIER_GRAPH_H_ */