#include <exploration_goal/frontier_graph.h>
#include <gtest/gtest.h>


class GraphCreateTest1 : public FrontierGraph
{
    public:
        GraphCreateTest1()
        {
            test_case = std::vector<double> { 0,0,0,
                                              1,0,0,
                                              2,0,0,
                                              2,1,0,
                                              0,2,0 };
            
            setGraph();

            adjacency_list.clear();
            adjacency_list.resize(num_nodes);
            graph.edges.clear();
            graph.edges.resize(num_nodes);

            setPointMapping();
            setAL();            
        }

        graph_msgs::GeometryGraph toMsg()
        {
            return graph;
        }

    protected:
        graph_msgs::GeometryGraph graph;
        std::vector<double> test_case;

        geometry_msgs::Point getPoint(double x, double y, double z)
        {
            geometry_msgs::Point tempPt;
            tempPt.x = x;
            tempPt.y = y;
            tempPt.z = z;

            return tempPt;
        }

        void setGraph()
        {
            ros::Time::init();
            graph.header.stamp = ros::Time::now();
            graph.header.frame_id = GLOBAL_TF_FRAME;
            
            num_nodes = test_case.size()/3;
            graph.nodes.resize(num_nodes);

            for (int idx=0;idx<num_nodes;idx++)
            {
                graph.nodes[idx] = getPoint(test_case[idx*3], test_case[idx*3+1], test_case[idx*3+2]);
            }

        }

        void setPointMapping()
        {
            node_idx_to_3d_point = graph.nodes;
        }

        void setAL()
        {
            adjacency_list[0].insert(1);
            adjacency_list[0].insert(4);
            adjacency_list[1].insert(2);
            adjacency_list[1].insert(3);

            //std::cout << graph << std::endl;

            graph.edges[0].node_ids.push_back(1);
            graph.edges[0].node_ids.push_back(4);
            graph.edges[1].node_ids.push_back(2);
            graph.edges[1].node_ids.push_back(3);
        }


};

class GraphCreateTest2 : public GraphCreateTest1
{
    public:
        GraphCreateTest2()
        {
            test_case = std::vector<double> { 0,2,0,
                                              0,2,1,  
                                              0,2,2,
                                              0,3,2  };     
            setGraph();

            adjacency_list.clear();
            adjacency_list.resize(num_nodes);
            graph.edges.clear();
            graph.edges.resize(num_nodes);

            setPointMapping();
            setAL();        
        }

        void setAL()
        {
            adjacency_list.resize(num_nodes);

            adjacency_list[0].insert(1);
            adjacency_list[1].insert(2);
            adjacency_list[1].insert(3);

            graph.edges.resize(num_nodes);

            graph.edges[0].node_ids.push_back(1);
            graph.edges[1].node_ids.push_back(2);
            graph.edges[1].node_ids.push_back(3);
        }
};

// Creates graph that is the result of merging GraphCreateTest1.merge(GraphCreateTest2)
class GraphMergeTest1 : public GraphCreateTest1
{
    public:
        GraphMergeTest1()
        {
            test_case = std::vector<double> { 0,0,0,
                                              1,0,0,
                                              2,0,0,
                                              2,1,0,
                                              0,2,0, 
                                              0,2,1,  
                                              0,2,2,
                                              0,3,2  };     

            setGraph();

            adjacency_list.clear();
            adjacency_list.resize(num_nodes);
            graph.edges.clear();
            graph.edges.resize(num_nodes);

            setPointMapping();
            setAL();        
        }

        void setAL()
        {
            adjacency_list.resize(num_nodes);

            adjacency_list[0].insert(1);
            adjacency_list[0].insert(4);
            adjacency_list[1].insert(2);
            adjacency_list[1].insert(3);

            adjacency_list[4].insert(5);
            adjacency_list[5].insert(6);
            adjacency_list[5].insert(7);

            graph.edges.resize(num_nodes);

            graph.edges[0].node_ids.push_back(1);
            graph.edges[1].node_ids.push_back(2);
            graph.edges[1].node_ids.push_back(3);

            graph.edges[4].node_ids.push_back(5);
            graph.edges[5].node_ids.push_back(6);
            graph.edges[5].node_ids.push_back(7);
        }
};

// To create new test case, just inherit base class GraphCreateTest1 and implemet constructor and setAL() function
// The vector<double> is used to set the mapping from node_idx to 3d geometry_msgs::Point. Adjacency list and graph_edges
// used to set the correct adjacency list against which the unit test can compare against

/*

    GraphTestCase()
        {
            test_case = std::vector<double> { 1,1,1,
                                              2,2,2,
                                              3,3,3  };     

            setGraph();

            adjacency_list.clear();
            adjacency_list.resize(num_nodes);
            graph.edges.clear();
            graph.edges.resize(num_nodes);

            setPointMapping();
            setAL();        
        }

        void setAL()
        {
            adjacency_list.resize(num_nodes);

            adjacency_list[0].insert(1);
            ......

            graph.edges.resize(num_nodes);

            graph.edges[0].node_ids.push_back(1);
            .....
        }





*/
