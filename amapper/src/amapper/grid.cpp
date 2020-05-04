#include <amapper/grid.h>

using namespace AMapper;

Grid::Grid() : Grid(0, 0, 2000, 2000, 1) {}

Grid::Grid(float xanchor, float yanchor, int16_t gridWidth, int16_t gridHeight, float resolution) {
    this->xAnchor = xanchor;
    this->yAnchor = yanchor;
    this->resolution = resolution;
    this->gridWidth = gridWidth;
    this->gridHeight = gridHeight;
    this->gridMetricWidth = gridWidth * resolution;
    this->gridMetricHeight = gridHeight * resolution;

    this->data = new int16_t*[this->gridHeight];
    for(int i = 0;i < this->gridHeight;i++){
        this->data[i] = new int16_t[this->gridWidth];
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = -1;
    }
    this->frameId = "";
}

Grid::Grid(const Grid& grid) {
    this->xAnchor = grid.xAnchor;
    this->yAnchor = grid.yAnchor;
    this->resolution = grid.resolution;
    this->gridWidth = grid.gridWidth;
    this->gridHeight = grid.gridHeight;
    this->gridMetricWidth = grid.gridWidth * resolution;
    this->gridMetricHeight = grid.gridHeight * resolution;

   this->data = new int16_t*[this->gridHeight];
    for(int i = 0;i < this->gridHeight;i++){
        this->data[i] = new int16_t[this->gridWidth];
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = grid.data[i][j];
    }
}

Grid::Grid(nav_msgs::OccupancyGrid occupancyGrid) {
    this->resolution = occupancyGrid.info.resolution;
    this->frameId = occupancyGrid.header.frame_id;
    this->gridWidth = occupancyGrid.info.width;
    this->gridHeight = occupancyGrid.info.height;
    this->gridMetricWidth = this->gridWidth * this->resolution;
    this->gridMetricHeight = this->gridHeight * this->resolution;
    this->xAnchor = occupancyGrid.info.origin.position.x + (this->gridWidth/2.0)*resolution;
    this->yAnchor = occupancyGrid.info.origin.position.y + (this->gridHeight/2.0)*resolution;
    
    this->data = new int16_t*[this->gridHeight];
    for(int i = 0;i < this->gridHeight;i++){
        this->data[i] = new int16_t[this->gridWidth];
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = occupancyGrid.data[i*gridHeight+j];
    }
}

void Grid::setFrameId(std::string frame_id){
    this->frameId = std::string(frame_id);
}

std::string Grid::getFrameId(){
    return this->frameId;
}

nav_msgs::OccupancyGrid Grid::toOccupancyGrid() {
    nav_msgs::OccupancyGrid occupancygrid;
    occupancygrid.header.frame_id = frameId;
    occupancygrid.header.stamp = ros::Time::now();
    occupancygrid.info.resolution = resolution;
    occupancygrid.info.width = this->gridWidth;
    occupancygrid.info.height = this->gridHeight;
    occupancygrid.info.origin.position.x = xAnchor - (this->gridWidth/2)*resolution;
    occupancygrid.info.origin.position.y = yAnchor - (this->gridHeight/2)*resolution;
    occupancygrid.info.origin.position.z = 0;
    occupancygrid.info.origin.orientation.x = 0;
    occupancygrid.info.origin.orientation.y = 0;
    occupancygrid.info.origin.orientation.z = 0;
    occupancygrid.info.origin.orientation.w = 1;
    for(int i = 0; i < this->gridHeight; i++){
        for(int j = 0; j < this->gridWidth; j++){
            occupancygrid.data.push_back((data[i][j] > 100 || data[i][j] < -1) ? 100: data[i][j] );
        }
    }
    return occupancygrid;
}

void Grid::clear() {
    for(int i =0;i < this->gridHeight;i++){
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = -1;
    }
}

Grid::~Grid() {
    for(int i =0;i < this->gridHeight;i++) {
        delete this->data[i];
    }
    delete this->data;
}

bool Grid::operator ==(const Grid &b) const{
    if(this->gridWidth != b.gridWidth){
        return false;
    }
    if(this->gridHeight != b.gridHeight){
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
    for(int i = 0; i < this->gridHeight; i++){
        for(int j = 0; j < this->gridWidth; j++){
            if(this->data[i][j] != b.data[i][j]){
                return false;
            }
        }
    }
    return true;
}