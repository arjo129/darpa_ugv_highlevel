#ifndef _ROS_THREAD_H_
#define _ROS_THREAD_H_
#include <ros/ros.h>
#include <QThread>
#include <sensor_msgs/LaserScan.h>
class ROSThread: public QThread {

    Q_OBJECT
    protected:
        void run() override;
    private:
        ros::NodeHandle nh;
        ros::Subscriber laserScanSub;
        bool running;
    public:
        ROSThread(ros::NodeHandle nh);
        ~ROSThread();
        void onLaserScan(sensor_msgs::LaserScan scan);
    signals:
        void scanRecieved(const QPixmap& map, int x, int y, float theta);
};
#endif

