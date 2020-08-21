#include <exploration_goal/frontier_graph.h>
#include "test_cases.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

void testFrontierGraphSize(FrontierGraph test_case, FrontierGraph ideal_case)
{
    ASSERT_EQ(test_case.getSize(), ideal_case.getSize()); 
}

void testFrontierGraphPointMapping(FrontierGraph test_case, FrontierGraph ideal_case)
{
    for (int node_idx=0;node_idx<test_case.getSize();node_idx++)
    {
        ASSERT_EQ(test_case.getPointForNodeId(node_idx), ideal_case.getPointForNodeId(node_idx));
    }
}

void testFrontierGraphAL(FrontierGraph test_case, FrontierGraph ideal_case)
{
    std::vector<std::set<int>> & test_case_al = test_case.getAL();
    std::vector<std::set<int>> & ideal_case_al = ideal_case.getAL();

    for (int node_idx=0;node_idx<test_case.getSize();node_idx++)
    {
        // Check edge list of every node
        ASSERT_EQ(test_case_al[node_idx], ideal_case_al[node_idx]);
    }
}

void testGraphEquality(FrontierGraph test_case, FrontierGraph ideal_case)
{
    // Check the number of nodes
    testFrontierGraphSize(test_case, ideal_case);


    // Check the index to 3d point mapping
    testFrontierGraphPointMapping(test_case, ideal_case);

    // Check adjacency list
    testFrontierGraphAL(test_case, ideal_case);
}

void testGraphInit(FrontierGraph ideal_case)
{
    FrontierGraph test_case(ideal_case.toMsg());

    testGraphEquality(test_case, ideal_case);
    
}

void testGraphMerge(FrontierGraph test_graph_parent, FrontierGraph test_graph_child, 
                                                            FrontierGraph ideal_case)
{
    test_graph_parent.mergeLocalGraph(test_graph_child);

    testGraphEquality(test_graph_parent, ideal_case);
}

TEST(FrontierGraph, empty_constructor)
{
    FrontierGraph graph;

    ASSERT_EQ(graph.getSize(), 0);
}

/**
 * Test if graph_msgs::GeometryGraph can be loaded into FrontierGraph Data Structure
 */ 
TEST(FrontierGraph, msg_constructor_1)
{
    GraphCreateTest1 ideal_case_1 = GraphCreateTest1();

    testGraphInit(ideal_case_1);
}

TEST(FrontierGraph, msg_constructor_2)
{
    GraphCreateTest2 ideal_case_2 = GraphCreateTest2();

    testGraphInit(ideal_case_2);
}


/**
 * Test merging of graph data structures
 */ 
TEST(FrontierGraph, merging_1)
{
    GraphCreateTest1 test_graph_parent = GraphCreateTest1();
    GraphCreateTest2 test_graph_child = GraphCreateTest2();
    GraphMergeTest1 ideal_case = GraphMergeTest1();

    testGraphMerge(test_graph_parent, test_graph_child, ideal_case);
}


int main(int argc, char **argv){
    //ros::init(argc, argv, "exploration_graph_load_test");
    testing::InitGoogleTest(&argc, argv);
    //ros::NodeHandle nh("~");
    return RUN_ALL_TESTS();
}