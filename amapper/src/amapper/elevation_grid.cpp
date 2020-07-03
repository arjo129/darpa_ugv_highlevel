#include <amapper/elevation_grid.h>

using namespace AMapper;

ElevationGrid::ElevationGrid(): ElevationGrid(0.05){
    
}

ElevationGrid::ElevationGrid(double res) {
    this->resolution = res;
}

ElevationGrid::ElevationGrid(const ElevationGrid& other) : data(other.data){
   this->resolution = other.resolution;
   this->frameId = other.frameId;
}

ElevationGrid::ElevationGrid(amapper::ElevationGridMsg elevationGrid) {
    this->setFrameId(elevationGrid.header.frame_id);
    this->resolution = elevationGrid.resolution;
    //Attempt to solve performance issues
    this->data.reserve(elevationGrid.data.size());
    for(auto point: elevationGrid.data) {
        this->add(point);
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
    
    elevationgrid.header.frame_id = this->getFrameId();
    elevationgrid.resolution = this->resolution;
    for(auto i: *this) {
        elevationgrid.data.push_back(i);
    }
    return elevationgrid;
}

void ElevationGrid::clear() {
    this->data.clear();
}

ElevationGrid::~ElevationGrid() {
}

bool ElevationGrid::operator ==(const ElevationGrid &other) const{
    
    if(this->frameId != other.frameId){
        return false;
    }

    if(this->resolution != other.resolution) {
        return false;
    }

    if(this->data != other.data) {
        return false;
    }

    return true;
}

ElevationIterator ElevationGrid::begin() {
    boost::shared_ptr<ElevationGrid> egrid = shared_from_this();
    return ElevationIterator(egrid, this->data.begin());
}

ElevationIterator ElevationGrid::end() {
    boost::shared_ptr<ElevationGrid> egrid = shared_from_this();
    return ElevationIterator(egrid, this->data.end());
} 