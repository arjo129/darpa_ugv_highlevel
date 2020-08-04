#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <exploration_goal/frontier_graph.h>
#include <exploration_goal/goal_selector.h>

#define LOCAL_GRAPH_TOPIC "graph"
#define GLOBAL_GRAPH_TOPIC "global_graph"
#define SELECTED_GOAL_TOPIC "goal_to_explore"
#define REQUEST_LOCAL_GRAPH_TOPIC "clicked_point"

/**
 * This node accepts local graphs produced by (ugv_obs_avoid pkg) frontier detection + path planning / branching and 
 * merges it to form a mega global graph and the next best goal is selected with DFS traversal for exploration.
 */  

ros::Publisher global_graph_pub, exploration_goal_pub, request_local_graph_pub;
FrontierGraph global_graph;

int main()
{
    ros::NodeHandle nh;
    ros::init(argc, argv,"exploration_goal_node");

    exploration_goal_pub = nh.advertise<geometry_msgs::PointStamped> >(SELECTED_GOAL_TOPIC, 1);
    global_graph_pub = nh.advertise<graph_msgs::GeometryGraph> >(GLOBAL_GRAPH_TOPIC, 1);
    request_local_graph_pub = nh.advertise<geometry_msgs::PointStamped> >(REQUEST_LOCAL_GRAPH_TOPIC, 1);

    while(ros::ok())
    {
        // request for local terrain graph
        geometry_msgs::PointStamped temp;
        request_local_graph_pub.publish(temp);

        // wait for local terrain graph to be received
        graph_msgs::GeometryGraphPtr local_graph;
        local_graph = ros::topic::waitForMessage<graph_msgs::GeometryGraph>(LOCAL_GRAPH_TOPIC, nh, ros::Duration(1.0));

        // Some frontiers are returned as local graph
        if (local_graph != NULL)
        {
            global_graph.mergeLocalGraph(*local_graph);
        }

        // choose best exploration goal from global graph
        int node_idx = get_exploration_goal(global_graph);
        Goal goal = global_graph[node_idx];

        exploration_goal_pub.publish(goal);

        // wait until robot stops moving / reaches goal
        wait_for_oscillation();

        global_graph.markAsVisited(node_idx);
    }

}