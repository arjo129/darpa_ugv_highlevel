#include <amapper/grid.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <set>

AMapper::Grid grid;
tf::TransformListener* listener;
std::vector<geometry_msgs::PointStamped> path;


struct CellDetail {
    int prev_idx_x, prev_idx_y;
    double g, h;
    bool init = false;
    double get_f() {
        if(init)
            return g+h;
        return INFINITY;
    }
};

typedef std::pair<double, std::pair<int, int>> GridPoint;

bool isValid(int row, int col) 
{ 
    return (row >= 0) && (row < grid.gridWidth) && (col >= 0) && (col < grid.gridHeight); 
} 
  
double euclideanDistance(int start_x, int start_y, int goal_x, int goal_y) {
    double r = (start_x - goal_x)*(start_x - goal_x);
    r += (start_y - goal_y)*(start_y - goal_y);
    return sqrt(r);
}

double euclideanDistance(geometry_msgs::PointStamped pt1, geometry_msgs::PointStamped pt2) {
    double r = (pt1.point.x - pt2.point.x)*(pt1.point.x - pt2.point.x) + (pt1.point.y - pt2.point.y)*(pt1.point.y - pt2.point.y);
    return sqrt(r);
}

void planRoute(size_t start_x, size_t start_y, size_t goal_x, size_t goal_y) {
    //Implementation of A*

    //Create empty cells
    CellDetail** cells = new CellDetail*[grid.gridHeight];
    for(int i = 0; i < grid.gridHeight; i++) {
        cells[i] = new CellDetail[grid.gridWidth];
    }

    //A* Algorithm
    std::set<GridPoint> open_list;
    cells[start_y][start_x].g = 0;
    cells[start_y][start_x].h = euclideanDistance(start_x, start_y, goal_x, goal_y);
    cells[start_y][start_x].init = true;

    open_list.insert(std::make_pair(cells[start_y][start_x].get_f(), std::make_pair(start_x, start_y)));
    
    size_t final_x, final_y;
    bool solution_found = false;
    while(!open_list.empty()) {
        auto current = *open_list.begin();
        auto x = current.second.first;
        auto y = current.second.second;
        open_list.erase(open_list.begin());
        if(euclideanDistance(x,y, goal_x, goal_y) < 3) {
            final_x = x;
            final_y = y;
            solution_found = true;
            break; 
        }
        for(int i = -1; i <= 1; i++) {
            for(int j = -1; j <= 1; j++) {
                auto curr_x = x + i, curr_y = y + j;
                if(!isValid(curr_y, curr_x)) continue;
                if(grid.data[curr_y][curr_x] != 0) continue; //TODO check surrounding area also
                auto tentative_score = current.first + 1;
                if(cells[curr_y][curr_x].g >= tentative_score) continue;
                cells[curr_y][curr_x].prev_idx_x = x;
                cells[curr_y][curr_x].prev_idx_y = y;
                cells[curr_y][curr_x].g = tentative_score;
                cells[curr_y][curr_x].h = euclideanDistance(curr_x, curr_y, goal_x, goal_y);
                cells[curr_y][curr_x].init = true;
                open_list.insert(std::make_pair(cells[curr_y][curr_x].get_f(),std::make_pair(curr_x, curr_y)));
            }
        }
    }

    if(!solution_found) {
        ROS_ERROR("A* could not find a solution!!");
        //Clean up
        for(int i =0;i < grid.gridHeight;i++) {
            delete cells[i];
        }
        delete cells;
        return;
    }

    auto curr_x = final_x;
    auto curr_y = final_y;
    path.clear();
    while(curr_x != start_x && curr_y != start_y) {
        geometry_msgs::PointStamped pt;
        pt.point.x = grid.fromXIndex(curr_x);
        pt.point.y = grid.fromYIndex(curr_y);
        pt.header.frame_id = grid.getFrameId();
        if(path.size() == 0) {
            path.push_back(pt);
        }
        else if(euclideanDistance(path[path.size()-1], pt) > 3) {
            path.push_back(pt);
        }
        auto next_x = cells[curr_y][curr_x].prev_idx_x;
        auto next_y = cells[curr_y][curr_x].prev_idx_y;
        curr_x = next_x;
        curr_y = next_y;
    }

    //Clean up
    for(int i =0;i < grid.gridHeight;i++) {
        delete cells[i];
    }
    delete cells;
}

void onRecieveNewPoint(geometry_msgs::PointStamped goal) {
    
    //Acquire robot pose
    tf::StampedTransform robot_pose;
    try {
        listener->waitForTransform("X1/base_link", grid.getFrameId(), goal.header.stamp, ros::Duration(1.0));
        listener->lookupTransform("X1/base_link", grid.getFrameId(), goal.header.stamp, robot_pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("Failed to transform node %s", ex.what());
        return;
    }

    if(goal.header.frame_id != grid.getFrameId()) {
        ROS_ERROR("Please pass the goal in %s frame for now", grid.getFrameId().c_str());
    }

    auto start_x = grid.toXIndex(robot_pose.getOrigin().x());
    auto start_y = grid.toYIndex(robot_pose.getOrigin().x());

    auto goal_x = grid.toXIndex(goal.point.x);
    auto goal_y = grid.toYIndex(goal.point.y);
}

void onRecieveMap(nav_msgs::OccupancyGrid occupancy_map){
    grid = AMapper::Grid(occupancy_map);
}

void onReachDestination(std_msgs::Int8 status) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh;
    listener = new tf::TransformListener;
    ros::Subscriber command_sub = nh.subscribe("/goal", 1, onRecieveNewPoint);
    ros::Subscriber map_sub = nh.subscribe("/global_map", 1, onRecieveMap);
    ros::Subscriber feedback_sub = nh.subscribe("/feedback", 1, onReachDestination);

    while(ros::ok()) {
        ros::spinOnce();
    }
} 