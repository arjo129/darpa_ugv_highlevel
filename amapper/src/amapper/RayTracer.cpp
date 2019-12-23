#include <amapper/RayTracer.h>

using namespace AMapper;

void RayTracer::rayTrace(Grid& grid, Eigen::Vector2f center, Eigen::Vector2f end) {

    int startX = grid.toXIndex(center.x());
    int startY = grid.toYIndex(center.y());
    int endX = grid.toXIndex(end.x());
    int endY = grid.toYIndex(end.y());
    float dX = endX-startX;
    float dY = endY-startY;
    float signdY = (dY < 0)? -1: 1;
    float signdX = (dX < 0)? -1: 1;  
    float deltaErr = fabs(dY/dX);
    float error = 0.0f;
    
    if(deltaErr > 1) {
        int x = startX;
        for(int y = startY; y != endY; y+=signdY){
            if(y >= 0 and x >= 0 and y < grid.gridSize and x < grid.gridSize){
                plotFreeSpace(grid, Eigen::Vector2i(startX, startY), x, y);
            }
                
            error+= 1/deltaErr;
            if(fabs(error) >= 0.5){
                x+=signdX;
                error-=1;
            }
        }
    }
    else{
        int y = startY;
        for(int x = startX; x != endX; x+=signdX) {     
            if(y >= 0 and x >= 0 and y < grid.gridSize and x < grid.gridSize)
                plotFreeSpace(grid, Eigen::Vector2i(startX, startY), x, y);
            error+= deltaErr;
            if(abs(error) >= 0.5){
                y+=signdY;
                error-=1;
            }
        }
    }
}
