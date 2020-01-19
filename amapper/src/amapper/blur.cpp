#include <amapper/grid.h>
#include <ros/ros.h>

namespace AMapper {
    Grid* inflate(Grid& grid, int radius) {
        Grid* result = new Grid(grid);
        result->setFrameId(grid.getFrameId());
        //ROS_INFO("New grid");
        for(int i = 0; i < grid.gridWidth; i++){
            for(int j = 0; j < grid.gridHeight; j++){

                if(grid.data[i][j] == 100){
                    for(int l = -radius; l <= radius; l++){
                        for(int m = -radius; m <= radius; m++){
                            if(grid.isWithinGridCellMap(i+l,j+m)){
                                //ROS_INFO("New grid %d, %d", i+l, j+m);
                                result->data[i+l][j+m] = 100;
                            }
                        }
                    }
                }
            }
        }
        return result;
    }

}