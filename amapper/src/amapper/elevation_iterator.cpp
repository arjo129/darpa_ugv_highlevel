#include <amapper/elevation_grid.h>

using namespace AMapper;

ElevationIterator::ElevationIterator(
    boost::shared_ptr<ElevationGrid> grid,
    CoordinateStorage::iterator iterator
):
egrid(grid)
{
    this->iterator = iterator;
}

amapper::ElevationPoint ElevationIterator::operator*() const {
    amapper::ElevationPoint pt;
    pt.elevation = this->iterator->second;
    pt.x = this->egrid->fromXIndex(this->iterator->first.first);
    pt.y = this->egrid->fromYIndex(this->iterator->first.second);
    return pt;
}

ElevationIterator& ElevationIterator::operator++() {
    ++this->iterator;
    return *this;
} 

bool ElevationIterator::operator==(const ElevationIterator& other) const {
    return this->iterator == other.iterator && this->egrid == other.egrid;
}