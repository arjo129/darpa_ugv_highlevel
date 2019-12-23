#ifndef _AMAPPER_DEVICE_LASER_
#define _AMAPPER_DEVICE_LASER_
#include <amapper/Device.h>
#include <amapper/RayTracer.h>
#include <sensor_msgs/LaserScan.h>

namespace AMapper {
    class LaserScanner : public Device<sensor_msgs::LaserScan>{
    private:
        RayTracer* raytracer;
    public:
        LaserScanner();
        void setRaytracer(RayTracer& rayTracer);
        void plotgrid(tf::Transform tf, Grid& grid, sensor_msgs::LaserScan& input);
    };
}
#endif