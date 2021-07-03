#include <ros/ros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/ColorRGBA.h>
#include <exploration_goal/frontier_graph.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <exploration_goal/frontier_graph.h>

#include <list>
#include <queue>
#include <cmath>
#include <set>
#include <float.h>
#include <stack>
#include <cstring>
#include <iterator>
#include <time.h>  

using namespace std;

#define GRAPH_LIFETIME 100000

struct cell {
    // ROW and COL index of the current's cell parent
    int parent;

    // f = g + h
    // h is known as heuristics
    double f, g, h;
};

class Astar {
private:
    int vertices;
    typedef pair<int, int> Pair;
    list<Pair> *adjList; // adjacency list
    graph_msgs::GeometryGraph graph_msg;

public:

    void create (const graph_msgs::GeometryGraph msg);

    // bi-directional
    // push_back: adds an element to the end 
    void addEdge(int u, int v, int w);

    void construct_graph_edges(const graph_msgs::GeometryGraph msg);

    // A utility funcion that ensures that the destination cell has been reached or not
    bool isDestination(int current, int destination);

    // A utility function that calculate the 'h' heuristics
    // In this case, the euclidean distance formula is used
    double calculateHvalue(int current, int destination, const graph_msgs::GeometryGraph graph_msg);

    // A utility function to trace the path from the source 
    // to destination 
    stack<int> tracePath(cell cellDetails[], int dest);

    stack<int> plan(int s, int d, const graph_msgs::GeometryGraph graph_msg);
};

void Astar::addEdge(int u, int v, int w) {
    adjList[u].push_back(make_pair(v, w));
    adjList[v].push_back(make_pair(u, w));
}

void Astar::construct_graph_edges(const graph_msgs::GeometryGraph msg) {
    for (int i=0; i<graph_msg.nodes.size(); ++i){
        for (int j=0; j< graph_msg.edges[i].node_ids.size() ; ++j){
            // double weight_g = calculateHvalue (source, destination, graph_msg);
            addEdge(i, graph_msg.edges[i].node_ids[j], 1); 
        }
    }
}

bool Astar::isDestination(int current, int destination) {
    if (current == destination) {
        return true;
    } else {
        return false;
    }
}

double Astar::calculateHvalue(int current, int destination, const graph_msgs::GeometryGraph graph_msg) {
    
    // return ((double)sqrt ((destination - current) * (destination - current)));

    double x1 = graph_msg.nodes[current].x;
    double y1 = graph_msg.nodes[current].y;
    double z1 = graph_msg.nodes[current].z;

    double x2 = graph_msg.nodes[destination].x;
    double y2 = graph_msg.nodes[destination].y;
    double z2 = graph_msg.nodes[destination].z;

    double distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2) + pow((z1 - z2), 2));
    return distance;
}

stack<int> Astar::tracePath(cell cellDetails[], int dest) { 
    int path = dest;
    stack<int> Path; //FILO
    
    // working backwards
    while (!(cellDetails[path].parent == path )) 
    { 
        Path.push (path); 
        int temp_path = cellDetails[path].parent; 
        path = temp_path; 
    } 

    // add the starting cell to the stack
    Path.push (path); 
    
    return Path; 
}

void Astar::create (const graph_msgs::GeometryGraph msg){
    vertices = msg.nodes.size();
    adjList = new list<Pair>[vertices];
    graph_msg = msg;

    construct_graph_edges(graph_msg);
}

stack<int> Astar::plan(int s, int d, const graph_msgs::GeometryGraph graph_msg) {

    // Create a closed list and initialise it to false which means 
    // that no cell has been included yet 
    // This closed list is implemented as a boolean 2D array 
    bool closedList[vertices]; 
    memset(closedList, false, sizeof (closedList)); 
    
    // Declare a 2D array of structure to hold the details 
    // of that cell 
    cell cellDetails[vertices];     

    int current; 

    // Similar to Djikstra - fill the cell with FLT_MAX initially
    for (int i = 0; i < vertices; i ++) {
        cellDetails[i].f = FLT_MAX; 
        cellDetails[i].g = FLT_MAX; 
        cellDetails[i].h = FLT_MAX; 
        cellDetails[i].parent = -1;
    } 

    /* 
    Create an open list having information as- 
    <f, current> 
    where f = g + h.
    This open list is implenented as a set of pair of pair.*/
    // This will act as the priority queue
    set<Pair> openList; 
    stack<int> path;

    // Put the starting cell on the open list and set its 
    // 'f' as 0 
    openList.insert(make_pair (0.0, s)); 

    cellDetails[s].f = 0.0; 
    cellDetails[s].g = 0.0; 
    cellDetails[s].h = 0.0;
    cellDetails[s].parent = s;

    // We set this boolean value as false as initially 
    // the destination is not reached. 
    bool foundDest = false;

    cout << endl << "Computing path using A*..." << endl;
    clock_t t = clock();

    while (!openList.empty()) 
    { 
        // Return the top/first value in the set (FIFO)
        Pair p = *openList.begin(); 

        // Remove this vertex from the open list 
        openList.erase(openList.begin()); 

        // Add this vertex to the closed list 
        current = p.second;
        closedList[current] = true; 

        double gNew, hNew, fNew; 

        // 'i' is used to get all adjacent vertices of a vertex 
        // check the neighbours
        list< pair<int, int> >::iterator i; 
        for (i = adjList[current].begin(); i != adjList[current].end(); ++i) 
        { 
            // Get vertex label and weight of current adjacent 
            // of u. 
            int v = (*i).first; 
            int weight = (*i).second; 

            if (isDestination(v, d)) {
                // Set the Parent of the destination cell 
                gNew = cellDetails[current].g + weight; // source to current
                hNew = calculateHvalue(v, d, graph_msg); // curr to goal
                fNew = gNew + hNew;
                // cout << v << " " << fNew << " " << hNew << "\n";
                cellDetails[v].f = fNew;
                cellDetails[v].g = gNew; 
                cellDetails[v].h = hNew; 
                cellDetails[v].parent = current; 
                printf ("The destination cell is found!\n"); 
                t = clock() - t;
                cout << "Took " << ((float)t)/CLOCKS_PER_SEC << " seconds.\n";
                path = tracePath (cellDetails, d);
                return path; 
            }

            if (closedList[v] == false) {
                gNew = cellDetails[current].g + weight;
                hNew = calculateHvalue(v, d, graph_msg);
                fNew = gNew + hNew;

                if (cellDetails[v].f == FLT_MAX || cellDetails[v].f > fNew) {
                    openList.insert(make_pair(fNew, v));
                    // Update the details of this cell
                    cellDetails[v].f = fNew;
                    cellDetails[v].g = gNew; 
                    cellDetails[v].h = hNew; 
                    cellDetails[v].parent = current; 
                }
                closedList[v] = true;
            }
        } 
    }
    return path;
}