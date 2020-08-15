#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <exploration_goal/frontier_graph.h>

#define LOCAL_GRAPH_TOPIC "graph"
#define GLOBAL_GRAPH_TOPIC "global_graph"
#define SELECTED_GOAL_TOPIC "goal_to_explore"
#define REQUEST_LOCAL_GRAPH_TOPIC "req_local_graph"
#define ROBOT_GOAL_STATUS "status"
#define START_EXPLORATION_TOPIC "start_exploration"

/**
 * This node accepts local graphs produced by (ugv_obs_avoid pkg) frontier detection + path planning / branching and 
 * merges it to form a mega global graph and the next best goal is selected with DFS traversal for exploration.
 */  

ros::Publisher global_graph_pub, exploration_goal_pub, request_local_graph_pub;
FrontierGraph global_graph;
boost::shared_ptr<ros::NodeHandle> nh;


void onStartCallback(const std_msgs::Empty e)
{
    ros::Rate loop_rate(1);
    while (ros::ok())
    {       
        if(global_graph.getAL().size() == 0 ||global_graph.isLeafNode()){

            ROS_INFO("Requesting Local Graph...");
            // request for local terrain graph
            geometry_msgs::PointStamped temp;
            temp.header.stamp = ros::Time::now();
            temp.header.frame_id = GLOBAL_TF_FRAME;
            request_local_graph_pub.publish(temp);

            // wait for local terrain graph to be received
            boost::shared_ptr<graph_msgs::GeometryGraph const> local_graph;
            local_graph = ros::topic::waitForMessage<graph_msgs::GeometryGraph>(LOCAL_GRAPH_TOPIC, *nh, ros::Duration(5.0));

            // Some frontiers are returned as local graph
            if (local_graph != NULL){
                FrontierGraph local_graph_obj(*local_graph);
                global_graph.mergeLocalGraph(local_graph_obj);
                // ROS_INFO("Global graph: ");
                // std::cout << global_graph.toMsg() << std::endl;
            }
        }

        global_graph_pub.publish(global_graph.toMsg());

        ROS_INFO("Get next goal...");
        geometry_msgs::PointStamped goal = global_graph.getNextGoal();
        exploration_goal_pub.publish(goal);
        ROS_INFO("Waiting for goal to be reached...");
        std_msgs::Int8ConstPtr status = ros::topic::waitForMessage<std_msgs::Int8>(ROBOT_GOAL_STATUS, *nh, ros::Duration(10.0));

        ROS_INFO("Received goal status...");

        if(status != NULL && status->data == 0){
            //reached goal
            global_graph.updateNewGoalSuccess();
        }else{
            global_graph.updateNewGoalFail();
            goal = global_graph.reset();
            exploration_goal_pub.publish(goal);
            status = ros::topic::waitForMessage<std_msgs::Int8>(ROBOT_GOAL_STATUS, *nh, ros::Duration(10.0));

        }
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv,"exploration_goal_node");
    nh.reset(new ros::NodeHandle());

    exploration_goal_pub = nh->advertise<geometry_msgs::PointStamped>(SELECTED_GOAL_TOPIC, 1);
    global_graph_pub = nh->advertise<graph_msgs::GeometryGraph>(GLOBAL_GRAPH_TOPIC, 1);
    request_local_graph_pub = nh->advertise<geometry_msgs::PointStamped>(REQUEST_LOCAL_GRAPH_TOPIC, 1);
    ros::Subscriber start_exploration = nh->subscribe(START_EXPLORATION_TOPIC, 1, onStartCallback);
    
    ros::spin();    

}
