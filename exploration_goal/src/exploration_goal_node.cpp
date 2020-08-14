#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <exploration_goal/frontier_graph.h>
#include <exploration_goal/goal_selector.h>

#define LOCAL_GRAPH_TOPIC "graph"
#define GLOBAL_GRAPH_TOPIC "global_graph"
#define SELECTED_GOAL_TOPIC "goal_to_explore"
#define REQUEST_LOCAL_GRAPH_TOPIC "req_local_graph"
#define ROBOT_GOAL_STATUS "status"

/**
 * This node accepts local graphs produced by (ugv_obs_avoid pkg) frontier detection + path planning / branching and 
 * merges it to form a mega global graph and the next best goal is selected with DFS traversal for exploration.
 */  

ros::Publisher global_graph_pub, exploration_goal_pub, request_local_graph_pub;
FrontierGraph global_graph;

int main(int argc, char *argv[])
{
    ros::NodeHandle nh;
    ros::init(argc, argv,"exploration_goal_node");

    exploration_goal_pub = nh.advertise<geometry_msgs::PointStamped>(SELECTED_GOAL_TOPIC, 1);
    global_graph_pub = nh.advertise<graph_msgs::GeometryGraph>(GLOBAL_GRAPH_TOPIC, 1);
    request_local_graph_pub = nh.advertise<geometry_msgs::PointStamped>(REQUEST_LOCAL_GRAPH_TOPIC, 1);
    
    while(ros::ok())
    {
        if(global_graph.getAL().size() == 0 ||global_graph.isLeafNode()){

               // request for local terrain graph
            geometry_msgs::PointStamped temp;
            request_local_graph_pub.publish(temp);

            // wait for local terrain graph to be received
            graph_msgs::GeometryGraphPtr local_graph;
            local_graph = ros::topic::waitForMessage<graph_msgs::GeometryGraph>(LOCAL_GRAPH_TOPIC, nh, ros::Duration(5.0));

            // Some frontiers are returned as local graph
            if (local_graph != NULL){
                FrontierGraph local_graph_obj(*local_graph);
                global_graph.mergeLocalGraph(local_graph_obj);
            }

        }

        geometry_msgs::PointStamped goal = global_graph.getNextGoal();
        exploration_goal_pub.publish(goal);
        int status = 0;
        // int status = ros::topic::waitForMessage<std_msgs::int8>(ROBOT_GOAL_STATUS, nh, ros::Duration(25.0));
        if(status == 0){
            //reached goal
            global_graph.updateNewGoalSuccess();
        }else{
            global_graph.updateNewGoalFail();
            goal = global_graph.reset();
            exploration_goal_pub.publish(goal);
            // int status = ros::topic::waitForMessage<std_msgs::int8>(ROBOT_GOAL_STATUS, nh, ros::Duration(25.0));

        }

    }

}