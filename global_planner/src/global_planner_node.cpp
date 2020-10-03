#include <amapper/grid.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <set>
#include <unordered_set>
#include <queue>
#include <std_msgs/Empty.h>
#include <chrono>
#include <sys/time.h>
#include <sys/resource.h>

AMapper::Grid* grid;
tf::TransformListener* listener;
std::vector<geometry_msgs::PointStamped> path;
ros::Publisher local_planner, next_location_req;
nav_msgs::OccupancyGrid latest_occupancy_map;


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

class InvalidLocationException: std::exception {
public:
    const char * what () const throw () {
      return "Target was too far from known map";
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

std::pair<size_t, size_t> findNearestEmptyPoint(size_t goal_x, size_t goal_y) {
    int dir[][2] = {{0,1},{1,0},{-1,0},{0,-1}, {1,1} ,{-1,-1}, {1,-1}, {-1,1}};
    std::queue<std::pair<size_t, size_t> > q;
    q.emplace(goal_x, goal_y);
    std::pair<size_t,size_t> target;
    while(!q.empty()) {
        auto x = q.front();
        q.pop();
        
        if(grid->data[x.second][x.first] == 0) {
            target = x;
            break;
        }

        if(euclideanDistance(x.first, x.second, goal_x, goal_y) > 5) {
            throw InvalidLocationException();
        }
        
        for(auto d: dir) {
            std::pair<size_t, size_t> next(x.first+d[0], x.second+d[1]);
            if(!isValid(next.first, next.second)) continue;
            q.push(next);
        }
    }
    return target;
}

void planRoute(size_t start_x, size_t start_y, size_t goal_x, size_t goal_y) {
    //Implementation of A*

    //Create empty cells
    CellDetail** cells = new CellDetail*[grid->gridHeight];
    for(int i = 0; i < grid->gridHeight; i++) {
        cells[i] = new CellDetail[grid->gridWidth];
    }

    bool** closed_set = new bool*[grid->gridHeight];
    for(int i = 0; i < grid->gridHeight; i++) {
        closed_set[i] = new bool[grid->gridWidth];
        for(int j = 0 ; j < grid->gridWidth; j++){
            closed_set[i][j] = false;
        }
    }


    //A* Algorithm
    ROS_INFO("Began Search for goal %d, %d", goal_x, goal_y);
    std::set<GridPoint> open_list;
    //std::unordered_set<std::pair<size_t, size_t>, pair_hash> closed_set;
    cells[start_y][start_x].g = 0;
    cells[start_y][start_x].h = euclideanDistance(start_x, start_y, goal_x, goal_y);
    cells[start_y][start_x].init = true;
    closed_set[start_y][start_x] = true;
    open_list.insert(std::make_pair(cells[start_y][start_x].get_f(), std::make_pair(start_x, start_y)));
    
    size_t final_x, final_y;
    bool solution_found = false;

    //Time out using system clock as ros clock uses sim time which may depend on resources
    auto start_plan = std::chrono::system_clock::now();
    std::chrono::seconds time_limit(10);

    while(!open_list.empty() && std::chrono::system_clock::now()-start_plan < time_limit) {
        auto current = *open_list.begin();
        auto x = current.second.first;
        auto y = current.second.second;
        open_list.erase(open_list.begin());
        closed_set[y][x] = true;
        ROS_DEBUG("Exploring %d %d", x,y);
        //closed_set.erase(std::make_pair(y,x));
        if(euclideanDistance(x,y, goal_x, goal_y) < 3) {
            ROS_DEBUG("Reached %d %d", x,y);

            final_x = x;
            final_y = y;
            solution_found = true;
            open_list.clear();
            break; 
        }
        int dir[][2] = {{0,1},{1,0},{-1,0},{0,-1}, {1,1} ,{-1,-1}, {1,-1}, {-1,1}};
        for(auto movement: dir) {
           
            auto curr_x = x + movement[0], curr_y = y + movement[1];

            if(!isValid(curr_y, curr_x)) {
                continue;
            }
            if(closed_set[curr_y][curr_x]) continue;
            if(grid->data[curr_y][curr_x] != 0) {
                closed_set[curr_y][curr_x] = true;
                ROS_DEBUG("Obstacle at %d %d", curr_x, curr_y);
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

    if(!solution_found) {
        ROS_ERROR("A* could not find a solution!!");
        ROS_ERROR("Reasons: ");
        ROS_ERROR("Destination color: %d", (int)grid->data[goal_y][goal_x]);
        ROS_ERROR("Current color: %d", (int)grid->data[start_y][start_x]);
        //ROS_ERROR("Time out: %f", );
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
        else if(euclideanDistance(path[path.size()-1], pt) > 1.5) {
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
    for(int i =0;i < grid->gridHeight;i++) {
        delete closed_set[i];
    }
    delete closed_set;
}

void executeRoute() {
    if(path.size() == 0) {
        std_msgs::Empty e;
        next_location_req.publish(e);
        ROS_ERROR("No path to execute");
        return;
    }
    ROS_INFO("Executing path");
    local_planner.publish(path[path.size()-1]);
    path.pop_back();
}

void debugPath() {

    for(auto p: path){
        std::cout << p <<std::endl;
    }
}

void onRecieveNewPoint(geometry_msgs::PointStamped goal) {
    ROS_INFO("Recieved new goal");
    //Acquire robot pose
    tf::StampedTransform robot_pose;
    try {
        listener->waitForTransform(grid->getFrameId(), "X1/base_link", goal.header.stamp, ros::Duration(1.0));
        listener->lookupTransform(grid->getFrameId(), "X1/base_link", goal.header.stamp, robot_pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("Failed to transform node %s", ex.what());
        return;
    }

    if(goal.header.frame_id != grid->getFrameId()) {
        ROS_ERROR("Please pass the goal in %s frame for now", grid->getFrameId().c_str());
        return;
    }
    ROS_INFO("Current Robot Pose %f %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y());
    auto start_x = grid->toXIndex(robot_pose.getOrigin().x());
    auto start_y = grid->toYIndex(robot_pose.getOrigin().y());

    auto goal_x = grid->toXIndex(goal.point.x);
    auto goal_y = grid->toYIndex(goal.point.y);

    ROS_INFO("Attempting to plan route");
    try{
        auto safest_spot = findNearestEmptyPoint(goal_x, goal_y);
        auto safest_start = findNearestEmptyPoint(start_x, start_y);
        planRoute(safest_start.first, safest_start.second, safest_spot.first, safest_spot.second);
        executeRoute();
        debugPath();
    } catch (InvalidLocationException il) {
        ROS_ERROR("Goal set is unreachable... Requesting new goal");
        std_msgs::Empty e;
        next_location_req.publish(e);
    }
}

void onRecieveMap(nav_msgs::OccupancyGrid occupancy_map){
    delete grid;
    grid = new AMapper::Grid(occupancy_map);
}

void onReachDestination(std_msgs::Int8 status) {
    if(path.size() == 0) {
        std_msgs::Empty e;
        next_location_req.publish(e);
        ROS_ERROR("No path to execute");
        return;
    }
    if(status.data < 0) {
        ROS_ERROR("Error on local planner");
        auto goal = path[0];
        //path.clear();
        goal.header.stamp = ros::Time::now();
        onRecieveNewPoint(goal);
        return;
    }
    local_planner.publish(path[path.size()-1]);
    path.pop_back();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");
    //getrlimit();
    ros::NodeHandle nh;
    listener = new tf::TransformListener;
    grid = new AMapper::Grid(0,0,20,20,1);
    ros::Subscriber command_sub = nh.subscribe("goal", 1, onRecieveNewPoint);
    ros::Subscriber map_sub = nh.subscribe("global_map", 1, onRecieveMap);
    ros::Subscriber feedback_sub = nh.subscribe("feedback", 1, onReachDestination);
    local_planner = nh.advertise<geometry_msgs::PointStamped>("local_plan", 1);
    next_location_req = nh.advertise<std_msgs::Empty>("explore/request", 1);
    ros::Rate r(10.0);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

/*
int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner_test");
    ros::NodeHandle nh;
    grid = new AMapper::Grid(0,0,20,20,1);
    for(int i =0;i < 20; i++){
        for(int j=0; j <20;j++)
        grid->data[i][j] = 0;
    }
     for(int j=1; j <20;j++) {
         grid->data[10][j] = 100;
     }
    planRoute(0, 0, 19, 19);

    for(auto p: path){
        std::cout << p <<std::endl;
    }
}*/