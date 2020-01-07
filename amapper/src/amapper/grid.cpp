#include <amapper/grid.h>

using namespace AMapper;

Grid::Grid() : Grid(0, 0, 2000, 1) {}

Grid::Grid(float xanchor, float yanchor, int gridsize, float resolution) {
    this->xAnchor = xanchor;
    this->yAnchor = yanchor;
    this->resolution = resolution;
    this->gridSize = gridsize;
    this->gridMetricSize = gridsize * resolution;

    this->data = new int16_t*[this->gridSize];
    for(int i =0;i < this->gridSize;i++){
        this->data[i] = new int16_t[this->gridSize];
        for(int j = 0; j < this->gridSize; j++) this->data[i][j] = -1;
    }
    this->frameId = "";
}

Grid::Grid(Grid& grid) {
    this->xAnchor = grid.xAnchor;
    this->yAnchor = grid.yAnchor;
    this->resolution = grid.resolution;
    this->gridSize = grid.gridSize;
    this->gridMetricSize = grid.gridSize * grid.resolution;

    this->data = new int16_t*[this->gridSize];
    for(int i =0;i < this->gridSize;i++){
        this->data[i] = new int16_t[this->gridSize];
        for(int j = 0; j < this->gridSize; j++) this->data[i][j] = grid.data[i][j];
    }
}

Grid::Grid(nav_msgs::OccupancyGrid occupancyGrid) {
    this->resolution = occupancyGrid.info.resolution;
    this->frameId = occupancyGrid.header.frame_id;
    this->gridSize = (int)round(sqrt(occupancyGrid.data.size()));
    this->gridMetricSize = this->gridSize * this->resolution;
    this->xAnchor = occupancyGrid.info.origin.position.x + this->gridSize/2*resolution;
    this->yAnchor = occupancyGrid.info.origin.position.y + this->gridSize/2*resolution;
    
    this->data = new int16_t*[this->gridSize];
    for(int i =0;i < this->gridSize;i++){
        this->data[i] = new int16_t[this->gridSize];
        for(int j = 0; j < this->gridSize; j++) this->data[i][j] = occupancyGrid.data[i*gridSize+j];
    }
}

void Grid::setFrameId(std::string frame_id){
    this->frameId = std::string(frame_id);
}

nav_msgs::OccupancyGrid Grid::toOccupancyGrid() {
    nav_msgs::OccupancyGrid occupancygrid;
    occupancygrid.header.frame_id = frameId;
    occupancygrid.header.stamp = ros::Time::now();
    occupancygrid.info.resolution = resolution;
    occupancygrid.info.width = this->gridSize;
    occupancygrid.info.height = this->gridSize;
    occupancygrid.info.origin.position.x = xAnchor - this->gridSize/2*resolution;
    occupancygrid.info.origin.position.y = yAnchor - this->gridSize/2*resolution;
    occupancygrid.info.origin.position.z = 0;
    occupancygrid.info.origin.orientation.x = 0;
    occupancygrid.info.origin.orientation.y = 0;
    occupancygrid.info.origin.orientation.z = 0;
    occupancygrid.info.origin.orientation.w = 1;
    for(int i = 0; i < this->gridSize; i++){
        for(int j = 0; j < this->gridSize; j++){
            occupancygrid.data.push_back((data[i][j] > 100 || data[i][j] < -1) ? 100: data[i][j] );
        }
    }
    return occupancygrid;
}

void Grid::clear() {
    for(int i =0;i < this->gridSize;i++){
        this->data[i] = new int16_t[this->gridSize];
        for(int j = 0; j < this->gridSize; j++) this->data[i][j] = -1;
    }
}

bool Grid::operator ==(const Grid &b) const{
    if(this->gridSize != b.gridSize){
        return false;
    }
    if(this->xAnchor != b.xAnchor){
        return false;
    }
    if(this->yAnchor != b.yAnchor){
        return false;
    }
    if(this->resolution != b.resolution){
        return false;
    }
    if(this->frameId != b.frameId){
        return false;
    }
    for(int i = 0; i < this->gridSize; i++){
        for(int j = 0; j < this->gridSize; j++){
            if(this->data[i][j] != b.data[i][j]){
                return false;
            }
        }
    }
    return true;
}