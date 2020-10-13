#include <ros/ros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <exploration_goal/frontier_graph.h>

// Reference: https://github.com/unr-arl/gbplanner_ros/blob/1530b24f9088be2951d91b9539d2fc6517359e06/gbplanner/src/gbplanner_rviz.cpp

#define GRAPH_LIFETIME 100000

ros::Publisher vis_pub;

visualization_msgs::Marker getEdgeMarkers(const graph_msgs::GeometryGraph graph_msg)
{
    // Plot all edges
    visualization_msgs::Marker edge_marker;
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.header.seq = 0;
    edge_marker.header.frame_id = graph_msg.header.frame_id;
    edge_marker.id = 0;
    edge_marker.ns = "edges";
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.scale.x = 0.04;
    edge_marker.color.r = 200.0 / 255.0;
    edge_marker.color.g = 100.0 / 255.0;
    edge_marker.color.b = 0.0;
    edge_marker.color.a = 1.0;
    edge_marker.lifetime = ros::Duration(GRAPH_LIFETIME);
    edge_marker.frame_locked = false;

    for (int parent_node_idx=0;parent_node_idx<graph_msg.edges.size();parent_node_idx++)
    {
        graph_msgs::Edges edge = graph_msg.edges[parent_node_idx];

        geometry_msgs::Point parent_node;
        parent_node = graph_msg.nodes[parent_node_idx];

        for (auto child_node_idx : edge.node_ids)
        {
            geometry_msgs::Point child_node;
            child_node = graph_msg.nodes[child_node_idx];

            // Line is drawn between every pair of points
            edge_marker.points.push_back(parent_node);
            edge_marker.points.push_back(child_node);
        }
    }

    return edge_marker;
}

visualization_msgs::Marker getVertexMarkers(const graph_msgs::GeometryGraph graph_msg)
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
    vertex_marker.color.a = 1.0;
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

void onGraphRecv(const graph_msgs::GeometryGraph graph_msg)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker edge_marker = getEdgeMarkers(graph_msg);
    visualization_msgs::Marker vertex_marker = getVertexMarkers(graph_msg);
    
    marker_array.markers.push_back(edge_marker);
    marker_array.markers.push_back(vertex_marker);

    ROS_INFO("[Viz] Graph Vertices and Edges Marker Array Published...");

    vis_pub.publish(marker_array);
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv,"global_exploration_graph_node");
    ros::NodeHandle nh;
    std::string graph_topic, graph_vis_topic;

    if (!(ros::param::has("~graph_topic")) || !(ros::param::has("~graph_vis_topic")))
    {
        ROS_ERROR("Graph or Graph Visualizing topics not configured (rosparam). Aborting graph" \
                                              "visualization node.");
        return -1;
    }

    ros::param::get("~graph_topic", graph_topic);
    ros::param::get("~graph_vis_topic", graph_vis_topic);

    ros::Subscriber graph_sub = nh.subscribe(graph_topic, 1, onGraphRecv);
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>(graph_vis_topic, 10);

    ros::spin();
}