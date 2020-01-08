#include <amapper/devices/laser_scanner.h>
#include <Eigen/Eigen>

using namespace AMapper;

LaserScanner::LaserScanner(){}

void LaserScanner::setRaytracer(RayTracer& rt){
    this->raytracer = &rt;
}

void LaserScanner::plotgrid(tf::Transform tf, Grid& grid, sensor_msgs::LaserScan& input){
    tf::Vector3 vec = tf.getOrigin();
    Eigen::Vector2f centroid(vec.x(), vec.y());
    tf::Quaternion qt = tf.getRotation();
    float yaw_offset = qt.getAngle();
    float beam_angle = input.angle_min;
    for(int i  = 0; i < input.ranges.size(); i++){
        if(input.ranges[i] > 20)
            continue;
        Eigen::Vector2f endPoint(input.ranges[i]*cos(beam_angle + yaw_offset), input.ranges[i]*sin(beam_angle + yaw_offset));
        raytracer->rayTrace(grid, centroid, endPoint);
        beam_angle += input.angle_increment;
        int endX = grid.toXIndex(endPoint.x());
        int endY = grid.toYIndex(endPoint.y());
        if(endY < grid.gridHeight && endX < grid.gridWidth&& endX >= 0 && endY >= 0)
        grid.data[endY][endX] = 100;
    }
}
