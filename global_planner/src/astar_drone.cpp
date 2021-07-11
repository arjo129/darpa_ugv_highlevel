#include "global_planner/astar_path_planning_drone.h"

/*
*  Take in a graph message (local currently, as leaking of nodes still occur in global graph) and plan path to a specified node
*/

bool first_local_graph_msg = true;

class GlobalPlanner{

private:
    ros::NodeHandle nh;
    ros::Publisher waypoints_pub, vis_pub;
    ros::Subscriber graph_sub, goal_sub;

    typedef pair<int, int> Pair;
    list<Pair> *adjList;
    int source;
    string GOAL_TOPIC, GLOBAL_GRAPH_TOPIC, GRAPH_VIS, PATH_TOPIC;
    bool new_goal;

    visualization_msgs::MarkerArray marker_array;
    graph_msgs::GeometryGraph global_graph_msg;

    Astar astar;

public:
    GlobalPlanner():
        nh("~")
    {
        ros::param::get("~goal_topic", GOAL_TOPIC);
        ros::param::get("~global_graph_topic", GLOBAL_GRAPH_TOPIC);
        ros::param::get("~graph_vis", GRAPH_VIS);
        ros::param::get("~path_topic", PATH_TOPIC);

        graph_sub = nh.subscribe(GLOBAL_GRAPH_TOPIC, 1, &GlobalPlanner::graphCallback, this);
        goal_sub = nh.subscribe(GOAL_TOPIC, 1, &GlobalPlanner::goalCallback, this);
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>(GRAPH_VIS, 10);
        waypoints_pub = nh.advertise<nav_msgs::Path>(PATH_TOPIC, 1);
        source = 0; 
        new_goal = true;
        
    }

    // construct graph (adjaceny list)
    void graphCallback(const graph_msgs::GeometryGraph graph_msg) {     
        int graph_size = graph_msg.nodes.size();
        global_graph_msg = graph_msg;

        // update global graph only when new goal has been assigned.
        // if (first_local_graph_msg && graph_size > 150){
        if (new_goal){
            new_goal = false;
            first_local_graph_msg = false;
            cout << "Graph (" << graph_size << " nodes) receieved." << endl;

            // create adjancey list for subsequent A* path planning
            astar.create(global_graph_msg);
            
            // visualization of graph_msg
            visualization_msgs::Marker vertex_marker = vis_VertexMarkers(global_graph_msg);       
            marker_array.markers.push_back(vertex_marker);
            vis_pub.publish(marker_array);
            cout << "Graph visualized on Rviz." << endl;
        }
    }

    // plan global path to goal using A* algorithm
    void goalCallback(const std_msgs::Int16 &goal){ //node_to_explore
        stack <int> path;
        new_goal = true;
        path = astar.plan(source, goal.data, global_graph_msg);

        visualization_msgs::Marker path_marker = vis_Path(global_graph_msg, path);
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(path_marker); 
        vis_pub.publish(marker_array);   
        source = goal.data;

        // Publishing planned path on PATH_TOPIC
        publishWaypoints(path, global_graph_msg);

    }

    visualization_msgs::Marker vis_VertexMarkers(const graph_msgs::GeometryGraph graph_msg)
    {
        // Plot all vertices
        visualization_msgs::Marker vertex_marker;
        vertex_marker.header.stamp = ros::Time::now();
        vertex_marker.header.seq = 0;
        vertex_marker.header.frame_id = graph_msg.header.frame_id;
        vertex_marker.id = 0;
        vertex_marker.ns = "vertices";
        vertex_marker.action = visualization_msgs::Marker::ADD;
        vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        vertex_marker.scale.x = 0.3;
        vertex_marker.scale.y = 0.3;
        vertex_marker.scale.z = 0.3;
        vertex_marker.color.r = 125.0 / 255.0;
        vertex_marker.color.g = 42.0 / 255.0;
        vertex_marker.color.b = 104.0 / 255.0;
        vertex_marker.color.a = 0.5;
        vertex_marker.lifetime = ros::Duration(GRAPH_LIFETIME);
        vertex_marker.frame_locked = false;

        for (int vertex_idx=0;vertex_idx<graph_msg.nodes.size();vertex_idx++)
        {
            vertex_marker.points.push_back(graph_msg.nodes[vertex_idx]);

            std_msgs::ColorRGBA color_msg;
            if (graph_msg.explored[vertex_idx] == GraphNodeExploredState::NODE_UNEXPLORED)
            {
                color_msg.r = 1.0;color_msg.g = 1.0;color_msg.b = 1.0;color_msg.a = 1.0; // white
            }
            else if (graph_msg.explored[vertex_idx] == GraphNodeExploredState::NODE_EXPLORED)
            {
                color_msg.r = 1.0;color_msg.g = 0.0;color_msg.b = 0.0;color_msg.a = 1.0; // red
            }
            else if (graph_msg.explored[vertex_idx] == GraphNodeExploredState::NODE_EXPLORING)
            {
                color_msg.r = 0.0;color_msg.g = 1.0;color_msg.b = 0.0;color_msg.a = 1.0; // green
            }
            else if (graph_msg.explored[vertex_idx] == GraphNodeExploredState::NODE_UNEXPLORABLE)
            {
                color_msg.r = 0.1;color_msg.g = 0.1;color_msg.b = 0.1;color_msg.a = 1.0; // green
            }


            vertex_marker.colors.push_back(color_msg);
        }
            
        return vertex_marker;
    }

 visualization_msgs::Marker vis_Path(const graph_msgs::GeometryGraph graph_msg, stack<int> Path)
    {
        // Plot all vertices
        visualization_msgs::Marker path_marker;
        path_marker.header.stamp = ros::Time::now();
        path_marker.header.seq = 0;
        path_marker.header.frame_id = graph_msg.header.frame_id;
        path_marker.id = 1;
        path_marker.ns = "path";
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.type = visualization_msgs::Marker::LINE_LIST;
        path_marker.scale.x = 0.2; // width
        path_marker.color.r = 225.0 / 255.0;
        path_marker.color.g = 42.0 / 255.0;
        path_marker.color.b = 104.0 / 255.0;
        path_marker.color.a = 1.0;
        path_marker.lifetime = ros::Duration(GRAPH_LIFETIME);
        path_marker.frame_locked = false;

        int size = Path.size();
        while (!Path.empty()) {  // "0 1 8 20 33 50" should become  "0-1, 1-8, 8-20, 20-33, 33-50"
            int p;
            geometry_msgs::Point node_id;
            
            for (int i = 0; i < size; ++i){
                p = Path.top(); 
                node_id = graph_msg.nodes[p];    
                
                if (i==0 || i == size -1 ){ // push back once for first and last node ID
                    path_marker.points.push_back(node_id);
                } else { // once for nodes in between
                    path_marker.points.push_back(node_id);
                    path_marker.points.push_back(node_id);
                }
                Path.pop(); 
            }
        }

        return path_marker;
    }

    void publishWaypoints (stack <int> node_indices, const graph_msgs::GeometryGraph graph_msg){
        
        geometry_msgs::PoseStamped pose;
        nav_msgs::Path path; 

        cout << "Path planned over " << node_indices.size() << " nodes." << endl;

        while (!node_indices.empty()) 
        { 
            int p = node_indices.top(); 
            node_indices.pop(); 

            pose.header.frame_id = graph_msg.header.frame_id;
            pose.pose.position = graph_msg.nodes[p];
            // orientation not given. will be handed by local planner
            pose.pose.orientation.x = 0; 
            pose.pose.orientation.y = 0; 
            pose.pose.orientation.z = 0; 
            pose.pose.orientation.w = 1; 

            path.poses.push_back(pose);
        }

        path.header.stamp = ros::Time::now();
        path.header.frame_id = graph_msg.header.frame_id;
        waypoints_pub.publish(path);
        cout << "Path visualized on Rviz." << endl;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv,"global_path_planning_node");

    ros::NodeHandle nh;
    GlobalPlanner Planner;
    
    ros::spin(); 
    
    return 0;
}