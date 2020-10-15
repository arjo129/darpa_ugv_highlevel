#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <exploration_goal/frontier_graph.h>

#define LOCAL_GRAPH_TOPIC "graph"
#define GLOBAL_GRAPH_TOPIC "global_graph"

/**
 * This node accepts local graphs produced by (ugv_obs_avoid pkg) frontier detection + path planning / branching and 
 * merges it to form a mega global graph. Node is mainly used to test global graph generation debugging.
 */  

ros::Publisher global_graph_pub;
boost::shared_ptr<FrontierGraph> global_graph;

void onLocalGraphRecv(const graph_msgs::GeometryGraph local_graph_msg)
{
    FrontierGraph local_graph(local_graph_msg);
    global_graph->mergeLocalGraph(local_graph);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv,"global_exploration_graph_node");
    ros::NodeHandle nh;

    ros::Subscriber local_graph_sub = nh.subscribe(LOCAL_GRAPH_TOPIC, 1, onLocalGraphRecv);
    global_graph_pub = nh.advertise<graph_msgs::GeometryGraph> (GLOBAL_GRAPH_TOPIC, 1);

    

    std::string robot_name;
    ros::param::get("~robot_name", robot_name);
    global_graph.reset(new FrontierGraph(robot_name+"/world"));

    ros::Rate loop_rate(5);

    while(ros::ok())
    {
        ros::spinOnce();
        global_graph_pub.publish(global_graph->toMsg());
        loop_rate.sleep();
    }

}