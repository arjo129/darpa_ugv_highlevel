#ifndef _AMAPPER_DEVICE_
#define _AMAPPER_DEVICE_

#include "grid.h"
#include "elevation_grid.h"
#include <tf/tf.h>

namespace AMapper {
    template<typename T>
    class Device {
    public:
        virtual void plotgrid(tf::Transform tf, Grid& grid, T input) {};
    };

    template<typename T>
    class ElevationDevice {
    public:
        virtual void plotgrid(tf::Transform tf, ElevationGrid& grid, T input) {};
    };

}
#endif