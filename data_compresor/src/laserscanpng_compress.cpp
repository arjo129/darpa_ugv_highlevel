#include <ros/ros.h>
#include <tf/tf.h>
#include <data_compressor/devices/laserscan.h>
#include <amapper/raytracers/NaiveRaytracer.h>
#include <amapper/devices/laser_scanner.h>
#include <opencv2/opencv.hpp>

AMapper::Grid* grid;
AMapper::LaserScanner ls;
AMapper::NaiveRaytracer rt;


cv::Mat gridToMat(const AMapper::Grid& grid){
    int size = grid.gridSize;
    cv::Mat mat = cv::Mat::zeros(cv::Size(size,size), CV_8UC1);
    for(int i = 0 ; i < size; i++){
        for(int j = 0; j > size; j++){
            if(grid.data[i][j] > 20)
            mat.at<uint8_t>(i,j) = 255;
        }
    }
    return mat;
}

cv::Mat scanToMat(sensor_msgs::LaserScan lscan){
    const int size = 500;
    float angle = lscan.angle_min;
    cv::Mat mat = cv::Mat::zeros(cv::Size(size,size), CV_8UC1);
    for(int i = 0; i < lscan.ranges.size(); i++){
        if(lscan.ranges[i] > lscan.range_max) {
             angle+=lscan.angle_increment;
             continue;
        }
        int x = round(size/2*(lscan.ranges[i]/(lscan.range_max))*cos(angle))+size/2;
        int y = round(size/2*(lscan.ranges[i]/(lscan.range_max))*sin(angle))+size/2;
        mat.at<uint8_t>(x,y) = 255;
        angle+=lscan.angle_increment;
    }
    return mat;
}
void onLaserScan (sensor_msgs::LaserScan lscan) {
    grid->clear();
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    ls.plotgrid(transform, *grid, lscan);
    cv::Mat mat = scanToMat(lscan);
    std::vector<uint8_t> buffer;
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    compression_params.push_back(0);
    cv::imencode(".png", mat, buffer, compression_params);
    //cv::imwrite("test.png", mat, compression_params);
    std::cout << "PNG encoding"<< buffer.size() <<std::endl;
    data_compressor::LaserScanCompressor compressor;
    data_compressor::DataPacket packet = compressor.compress(lscan);
    std::cout << "Zip encoding "<< packet.data.size() <<std::endl;
}

int main(int argc, char** argv) {
    grid = new AMapper::Grid(0,0,1000,0.05);
    grid->setFrameId(std::string("laser"));
    ls.setRaytracer(rt);
    ros::init(argc, argv, "LaserScanExampleNode");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 1, onLaserScan);
    while(ros::ok()) {
        ros::spinOnce();
    }
}