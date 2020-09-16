#include <amapper/grid.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <set>
#include <unordered_set>

AMapper::Grid* grid;
tf::TransformListener* listener;
std::vector<geometry_msgs::PointStamped> path;
ros::Publisher local_planner;

struct CellDetail {
    int prev_idx_x, prev_idx_y;
    double g = INFINITY, h=INFINITY;
    bool init = false;
    double get_f() {
        if(init)
            return g+h;
        return INFINITY;
    }
};

typedef std::pair<double, std::pair<int, int>> GridPoint;

struct pair_hash
{
	template <class T1, class T2>
	std::size_t operator () (std::pair<T1, T2> const &pair) const
	{
		std::size_t h1 = std::hash<T1>()(pair.first);
		std::size_t h2 = std::hash<T2>()(pair.second);

		return h1 ^ h2;
	}
};

bool isValid(int row, int col) 
{ 
    return (row >= 0) && (row < grid->gridWidth) && (col >= 0) && (col < grid->gridHeight); 
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
    CellDetail** cells = new CellDetail*[grid->gridHeight];
    for(int i = 0; i < grid->gridHeight; i++) {
        cells[i] = new CellDetail[grid->gridWidth];
    }

    //A* Algorithm
    ROS_INFO("Began Search for goal %d, %d", goal_x, goal_y);
    std::set<GridPoint> open_list;
    std::unordered_set<std::pair<size_t, size_t>, pair_hash> closed_set;
    cells[start_y][start_x].g = 0;
    cells[start_y][start_x].h = euclideanDistance(start_x, start_y, goal_x, goal_y);
    cells[start_y][start_x].init = true;
    closed_set.emplace(start_y, start_x);
    open_list.insert(std::make_pair(cells[start_y][start_x].get_f(), std::make_pair(start_x, start_y)));
    
    size_t final_x, final_y;
    bool solution_found = false;
    auto start_plan = ros::Time::now();
    
    while(!open_list.empty() && ros::Time::now()-start_plan > ros::Duration(30)) {
        auto current = *open_list.begin();
        auto x = current.second.first;
        auto y = current.second.second;
        open_list.erase(open_list.begin());
        closed_set.emplace(y, x);
        //closed_set.erase(std::make_pair(y,x));
        if(euclideanDistance(x,y, goal_x, goal_y) < 5) {
            ROS_INFO("Reached %d %d", x,y);

            final_x = x;
            final_y = y;
            solution_found = true;
            open_list.clear();
            break; 
        }
        for(int i = -1; i <= 1; i++) {
            for(int j = -1; j <= 1; j++) {
                auto curr_x = x + i, curr_y = y + j;

                if(!isValid(curr_y, curr_x) || (i ==0 && j == 0)) {
                    continue;
                }
                if(closed_set.count(std::make_pair(curr_y,curr_x))>0) continue;
                if(grid->data[curr_y][curr_x] != 0) {
                    closed_set.emplace(curr_y,curr_x);
                    continue; //TODO check surrounding area also
                }
                auto tentative_score = cells[y][x].g + 1; 
                if(cells[curr_y][curr_x].g >= tentative_score && cells[curr_y][curr_x].init) continue;
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
        for(int i =0;i < grid->gridHeight;i++) {
            delete cells[i];
        }
        delete cells;
        return;
    }

    auto curr_x = final_x;
    auto curr_y = final_y;
    path.clear();
    ROS_INFO("Found route, tracing back");
    while(curr_x != start_x && curr_y != start_y) {
        geometry_msgs::PointStamped pt;
        pt.point.x = grid->fromXIndex(curr_x);
        pt.point.y = grid->fromYIndex(curr_y);
        pt.header.frame_id = grid->getFrameId();
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
    ROS_INFO("Finished traceback");
    //Clean up
    for(int i =0;i < grid->gridHeight;i++) {
        delete cells[i];
    }
    delete cells;
}

void executeRoute() {
    if(path.size() == 0) {
        ROS_ERROR("No path to execute");
        return;
    }
    ROS_INFO("Executing path");
    local_planner.publish(path[path.size()-1]);
    path.pop_back();
}

void onRecieveNewPoint(geometry_msgs::PointStamped goal) {
    
    //Acquire robot pose
    tf::StampedTransform robot_pose;
    try {
        listener->waitForTransform("X1/base_link", grid->getFrameId(), goal.header.stamp, ros::Duration(1.0));
        listener->lookupTransform("X1/base_link", grid->getFrameId(), goal.header.stamp, robot_pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("Failed to transform node %s", ex.what());
        return;
    }

    if(goal.header.frame_id != grid->getFrameId()) {
        ROS_ERROR("Please pass the goal in %s frame for now", grid->getFrameId().c_str());
        return;
    }

    auto start_x = grid->toXIndex(robot_pose.getOrigin().x());
    auto start_y = grid->toYIndex(robot_pose.getOrigin().x());

    auto goal_x = grid->toXIndex(goal.point.x);
    auto goal_y = grid->toYIndex(goal.point.y);

    ROS_INFO("Attempting to plan route");
    planRoute(start_x, start_y, goal_x, goal_y);
    executeRoute();
}

void onRecieveMap(nav_msgs::OccupancyGrid occupancy_map){
    delete grid;
    grid = new AMapper::Grid(occupancy_map);
    ROS_INFO("Recieved occupancy grid");
}

void onReachDestination(std_msgs::Int8 status) {
    if(path.size() == 0) {
        ROS_ERROR("No path to execute");
        return;
    }
    local_planner.publish(path[path.size()-1]);
    path.pop_back();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh;
    listener = new tf::TransformListener;
    grid = new AMapper::Grid(0,0,20,20,1);
    ros::Subscriber command_sub = nh.subscribe("goal", 1, onRecieveNewPoint);
    ros::Subscriber map_sub = nh.subscribe("global_map", 1, onRecieveMap);
    ros::Subscriber feedback_sub = nh.subscribe("feedback", 1, onReachDestination);
    local_planner = nh.advertise<geometry_msgs::PointStamped>("local_plan", 1);

    while(ros::ok()) {
        ros::spinOnce();
    }
}

/*
int main(int argc, char** argv) {
    grid = new AMapper::Grid(0,0,20,20,1);
    for(int i =0;i < 20; i++){
        for(int j=0; j <20;j++)
        grid->data[i][j] = 0;
    }
    planRoute(0, 0, 9, 9);
}*/