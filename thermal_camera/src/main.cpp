#include "../include/ThermalPointCloudMapper.h"

#include <functional>

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_mapper");

    ros::Time::init();
    ros::NodeHandle nh;
    ThermalPointCloudMapper mapper;

    ros::spin();
}