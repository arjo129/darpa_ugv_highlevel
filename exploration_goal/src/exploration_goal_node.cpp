#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <exploration_goal/frontier_graph.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#define LOCAL_GRAPH_TOPIC "graph"
#define GLOBAL_GRAPH_TOPIC "global_graph"
#define SELECTED_GOAL_TOPIC "goal_to_explore"
#define REQUEST_LOCAL_GRAPH_TOPIC "req_local_graph"
#define ROBOT_GOAL_STATUS "status"
#define START_EXPLORATION_TOPIC "start_exploration"
#define PATH_TOPIC "waypoints"
#define WAY_POINTS_TOPIC "waypoints"
/**
 * This node accepts local graphs produced by (ugv_obs_avoid pkg) frontier detection + path planning / branching and 
 * merges it to form a mega global graph and the next best goal is selected with DFS traversal for exploration.
 */  

ros::Publisher global_graph_pub, exploration_goal_pub, request_local_graph_pub, waypoints_pub;
boost::shared_ptr<FrontierGraph> global_graph;
boost::shared_ptr<ros::NodeHandle> nh;
std::string robot_name;
nav_msgs::Path Waypoints;





// void onWayPointsRecv(const nav_msgs::Path waypoints)
// {
    
//     ROS_INFO("RECEIVED WAYPOINTS!!!" , waypoints.poses.size());
// }


void onHeadStartWaypoint(const nav_msgs::Path p) {
    global_graph->addWayPoints(p);
}


void onStartCallback(const std_msgs::Empty e)
{
    ros::Rate loop_rate(1);
    bool stuck = false;
    ros::param::get("~robot_name", robot_name);
    Waypoints.header.frame_id = robot_name+"/world";


    while (ros::ok())
    {       
        if(global_graph->getAL().size() == 0 ||global_graph->isLeafNode() || stuck){
            stuck = false;

            ROS_INFO("Requesting Local Graph...");
            // request for local terrain graph
            geometry_msgs::PointStamped temp;
            temp.header.stamp = ros::Time::now();
            temp.header.frame_id = robot_name;
            request_local_graph_pub.publish(temp);

            // wait for local terrain graph to be received
            boost::shared_ptr<graph_msgs::GeometryGraph const> local_graph;
            local_graph = ros::topic::waitForMessage<graph_msgs::GeometryGraph>(LOCAL_GRAPH_TOPIC, *nh, ros::Duration(5.0));

            // Some frontiers are returned as local graph
            if (local_graph != NULL){
                FrontierGraph local_graph_obj(*local_graph);
                global_graph->mergeLocalGraph(local_graph_obj);
                // ROS_INFO("Global graph: ");
                // std::cout << global_graph->toMsg() << std::endl;
            }
        }

        global_graph_pub.publish(global_graph->toMsg());

        ROS_INFO("Get next goal...");
        geometry_msgs::PointStamped goal = global_graph->getNextGoal();
        geometry_msgs::PoseStamped goalP;
        goalP.pose.position = goal.point;
        goalP.header.frame_id = robot_name+"/world";
        goalP.header.stamp = ros::Time::now();
        exploration_goal_pub.publish(goal);
        ROS_INFO("Waiting for goal to be reached...");
        std_msgs::Int8ConstPtr status = ros::topic::waitForMessage<std_msgs::Int8>(ROBOT_GOAL_STATUS, *nh, ros::Duration(20.0));

        ROS_INFO("Received goal status...");

        if(status != NULL && status->data == 0){
            ROS_INFO("[Exploration] Reached goal node successfully...");

            global_graph->updateNewGoalSuccess();
            global_graph_pub.publish(global_graph->toMsg());
            Waypoints.header.stamp = ros::Time::now();
            Waypoints.poses.push_back(goalP);
            //waypoints_pub.publish(Waypoints);
        }else{
            ROS_ERROR("[Exploration] Failed to reach goal node...");
            global_graph->updateNewGoalFail();
            // goal = global_graph->reset();
            // exploration_goal_pub.publish(goal);
            stuck = true;
            global_graph_pub.publish(global_graph->toMsg());
            // ROS_INFO("[Exploration] Going back to parent node...");
            // status = ros::topic::waitForMessage<std_msgs::Int8>(ROBOT_GOAL_STATUS, *nh, ros::Duration(10.0));
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

    //waypoints_pub = nh->advertise<nav_msgs::Path>(PATH_TOPIC, 1);

    ros::Subscriber start_exploration = nh->subscribe(START_EXPLORATION_TOPIC, 1, onStartCallback);
    ros::Subscriber wayPointsSub = nh->subscribe(WAY_POINTS_TOPIC, 1, onHeadStartWaypoint);
    
    ros::param::get("~robot_name", robot_name);
    Waypoints.header.frame_id = robot_name+"/world";
    global_graph.reset(new FrontierGraph(robot_name+"/world"));
    
    ros::spin();    

}
