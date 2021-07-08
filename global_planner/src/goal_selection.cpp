/*
*  Selection of goal which fulfils the following criteria:
*  - unexplored
*  - in front of robot's heading (to ensure robot traverse inwards into cave)
*/

#include <ros/ros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>
#include <std_msgs/Int16.h>

using namespace std;

class goalSelector{

private:
    ros::NodeHandle nh;
    ros::Publisher goal_to_explore_pub;
    ros::Subscriber graph_sub;

    string GLOBAL_GRAPH_TOPIC, GOAL_TOPIC;
public:
    goalSelector():
        nh("~")
    {
        ros::param::get("~global_graph_topic", GLOBAL_GRAPH_TOPIC);
        ros::param::get("~goal_topic", GOAL_TOPIC);        

        graph_sub = nh.subscribe(GLOBAL_GRAPH_TOPIC, 1, &goalSelector::graphCallback, this);
        goal_to_explore_pub = nh.advertise<std_msgs::Int16>(GOAL_TOPIC, 10);
    }

    // filling in explored nodes manually (wait for tze guang's part)
    void graphCallback(const graph_msgs::GeometryGraph::Ptr &graph_msg) {     




        // select_goal(graph_msg); // pass pointer
        select_goal(*graph_msg); // pass content


    }

    // void select_goal (const graph_msgs::GeometryGraph::Ptr&){ //pass pointer

    // }

    void select_goal (graph_msgs::GeometryGraph graph_msg){ // receive content of graph

    }



};

int main(int argc, char** argv)
{
    ros::init(argc, argv,"goal_selection");

    ros::NodeHandle nh;
    goalSelector goalSelector;
    
    ros::spin(); 
    
    return 0;
}