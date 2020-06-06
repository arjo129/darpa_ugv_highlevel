#include <ros/ros.h>
#include <gtest/gtest.h>
#include <amapper/grid.h>
#include <amapper/elevation_grid.h>

TEST(AmapperGrid, equalityOperatorTest)
{
  AMapper::Grid grid1, grid2;
  //Happy case
  ASSERT_TRUE(grid1 == grid2);

  //Change data
  grid1.data[1][1] = 25;
  ASSERT_FALSE(grid1 == grid2);
}

TEST(AmapperGrid, indexConversionsConsistent)
{

}

TEST(AmapperGrid, occupancygridConversionConsistent)
{
  AMapper::Grid grid1;
  grid1.data[1][1] = 25;
  AMapper::Grid grid2(grid1.toOccupancyGrid());

  ASSERT_TRUE(grid1 == grid2);

}

TEST(AmapperGrid, gridClearsCorrectly)
{
  AMapper::Grid grid1, emptyGrid;
  grid1.data[1][1] = 25;
  grid1.clear();
  ASSERT_TRUE(grid1 == emptyGrid);
}

TEST(AmapperGrid, setsFrameIdCorrectly)
{
  AMapper::Grid grid1;
  grid1.setFrameId("test");
  nav_msgs::OccupancyGrid out = grid1.toOccupancyGrid();

  ASSERT_TRUE(out.header.frame_id == "test");

}

TEST(AmapperElevationGrid, recallsAllFrames) {

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}