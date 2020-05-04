#include <amapper/elevation_grid.h>

using namespace AMapper;

ElevationGrid::ElevationGrid() : ElevationGrid(0, 0, 2000, 2000, 1) {}

ElevationGrid::ElevationGrid(float xanchor, float yanchor, int16_t gridWidth, int16_t gridHeight, float resolution) {
    this->xAnchor = xanchor;
    this->yAnchor = yanchor;
    this->resolution = resolution;
    this->gridWidth = gridWidth;
    this->gridHeight = gridHeight;
    this->gridMetricWidth = gridWidth * resolution;
    this->gridMetricHeight = gridHeight * resolution;

    this->data = new double*[this->gridHeight];
    for(int i = 0;i < this->gridHeight;i++){
        this->data[i] = new double[this->gridWidth];
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = -INFINITY;
    }
    this->frameId = "";
}

ElevationGrid::ElevationGrid(const ElevationGrid& grid) {
    this->xAnchor = grid.xAnchor;
    this->yAnchor = grid.yAnchor;
    this->resolution = grid.resolution;
    this->gridWidth = grid.gridWidth;
    this->gridHeight = grid.gridHeight;
    this->gridMetricWidth = grid.gridWidth * resolution;
    this->gridMetricHeight = grid.gridHeight * resolution;

   this->data = new double*[this->gridHeight];
    for(int i = 0;i < this->gridHeight;i++){
        this->data[i] = new double[this->gridWidth];
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = grid.data[i][j];
    }
}

ElevationGrid::ElevationGrid(amapper::ElevationGridMsg elevationGrid) {
    this->resolution = elevationGrid.info.resolution;
    this->frameId = elevationGrid.header.frame_id;
    this->gridWidth = elevationGrid.info.width;
    this->gridHeight = elevationGrid.info.height;
    this->gridMetricWidth = this->gridWidth * this->resolution;
    this->gridMetricHeight = this->gridHeight * this->resolution;
    this->xAnchor = elevationGrid.info.origin.position.x + (this->gridWidth/2.0)*resolution;
    this->yAnchor = elevationGrid.info.origin.position.y + (this->gridHeight/2.0)*resolution;
    
    this->data = new double*[this->gridHeight];
    for(int i = 0;i < this->gridHeight;i++){
        this->data[i] = new double[this->gridWidth];
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = elevationGrid.data[i*gridHeight+j];
    }
}

void ElevationGrid::ElevationGrid::setFrameId(std::string frame_id){
    this->frameId = std::string(frame_id);
}

std::string ElevationGrid::getFrameId(){
    return this->frameId;
}

amapper::ElevationGridMsg ElevationGrid::toRosMsg() {
    amapper::ElevationGridMsg elevationgrid;
    elevationgrid.header.frame_id = frameId;
    elevationgrid.header.stamp = ros::Time::now();
    elevationgrid.info.resolution = resolution;
    elevationgrid.info.width = this->gridWidth;
    elevationgrid.info.height = this->gridHeight;
    elevationgrid.info.origin.position.x = xAnchor - (this->gridWidth/2)*resolution;
    elevationgrid.info.origin.position.y = yAnchor - (this->gridHeight/2)*resolution;
    elevationgrid.info.origin.position.z = 0;
    elevationgrid.info.origin.orientation.x = 0;
    elevationgrid.info.origin.orientation.y = 0;
    elevationgrid.info.origin.orientation.z = 0;
    elevationgrid.info.origin.orientation.w = 1;
    for(int i = 0; i < this->gridHeight; i++){
        for(int j = 0; j < this->gridWidth; j++){
            elevationgrid.data.push_back( data[i][j] );
        }
    }
    return elevationgrid;
}

void ElevationGrid::clear() {
    for(int i =0;i < this->gridHeight;i++) {
        for(int j = 0; j < this->gridWidth; j++) this->data[i][j] = -INFINITY;
    }
}

ElevationGrid::~ElevationGrid() {
    for(int i =0;i < this->gridHeight;i++) {
        delete this->data[i];
    }
    delete this->data;
}

bool ElevationGrid::operator ==(const ElevationGrid &b) const{
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