#ifndef _AMAPPER_ELEVATION_GRID_H_
#define _AMAPPER_ELEVATION_GRID_H_

#include <amapper/ElevationGridMsg.h>
#include <amapper/ElevationPoint.h>
#include <math.h>
#include <unordered_map>
#include <boost/enable_shared_from_this.hpp>


namespace AMapper {
    class ElevationIterator ;

    /**
     * Utility to hash pairs
     */ 
    struct hash_pair { 
        template <class T1, class T2> 
        size_t operator()(const std::pair<T1, T2>& p) const
        { 
            auto hash1 = std::hash<T1>{}(p.first); 
            auto hash2 = std::hash<T2>{}(p.second); 
            return hash1 ^ hash2; 
        } 
    };

    typedef std::pair<long,long> Coordinate;
    template<typename T>
    using GenericCoordinateStorage = std::unordered_map<std::pair<long,long>, T, hash_pair> ;
    typedef GenericCoordinateStorage<double> CoordinateStorage;
   
    /**
     * ElevationGrid class: Essentially stores a grid. 
     */ 
    class ElevationGrid: public boost::enable_shared_from_this<ElevationGrid> {
    private:
        double resolution; ///Units [m/cell]
        std::string frameId;
        CoordinateStorage data;
    public:
        ElevationGrid();
        ElevationGrid(double res);
        ElevationGrid(const ElevationGrid& grid);
        ElevationGrid(amapper::ElevationGridMsg occupancyGrid);
        ~ElevationGrid();

        /**
         * Set the tf frame in which grid is in
         */ 
        void setFrameId(std::string frame_id);
        /**
         * Set the tf frame in which grid is in
         */
        std::string getFrameId();
        
        /**
         * Convert to occupancy grid
         */ 
        amapper::ElevationGridMsg toRosMsg();


        /**
         * Some helper functions to help with grid indexing
         */
        inline long toYIndex(double yValue) {
            return (long)round(yValue/resolution);
        }

        inline long toXIndex(double xValue) {
            return (long)round(xValue/resolution);
        }

        inline float fromXIndex(int xValue) {
            return xValue*resolution;
        }

        inline float fromYIndex(int yValue) {
            return yValue*resolution;
        }

        /**
         * Set elevation at point
         */ 
        inline void add(amapper::ElevationPoint pt) {
            std::pair<long, long> coordinate(toXIndex(pt.x), toYIndex(pt.y));
            data[coordinate] = pt.elevation;
        }

        inline double queryElevation(double x, double y) {
            std::pair<long, long> coordinate(toXIndex(x), toYIndex(y));
            if(data.count(coordinate) != 0)
                return data[coordinate];
            return -INFINITY;
        }

        double getResolution() {
            return resolution;
        }

        void clear();

        bool operator ==(const ElevationGrid &b) const;

        ElevationIterator begin();
        ElevationIterator end();
    };
     /**
     * This class makes it easy to iterate over the ElevationGrid class
     */ 
    class ElevationIterator {
    private:
        CoordinateStorage::iterator iterator;
        boost::shared_ptr<ElevationGrid> egrid;
    public:
        typedef amapper::ElevationPoint     value_type;
        typedef std::ptrdiff_t              difference_type;
        typedef amapper::ElevationPoint*    pointer;
        typedef amapper::ElevationPoint&    reference;
        typedef std::input_iterator_tag iterator_category;
        ElevationIterator(
            boost::shared_ptr<ElevationGrid> grid,
            CoordinateStorage::iterator top_level
        );
        bool operator==(const ElevationIterator& other) const;
        bool operator!=(const ElevationIterator& other) const { return !(*this == other); };
        amapper::ElevationPoint operator*() const;
        ElevationIterator& operator++();
    };
    
};

#endif