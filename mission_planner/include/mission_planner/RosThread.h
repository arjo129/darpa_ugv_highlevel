#ifndef _ROS_THREAD_H_
#define _ROS_THREAD_H_
#include <ros/ros.h>
#include <QThread>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

class ROSThread: public QThread {

    Q_OBJECT
    protected:
        void run() override;
    private:
        ros::NodeHandle nh;
        ros::Subscriber laserScanSub;
        ros::Subscriber odometrySub;
        nav_msgs::Odometry recentOdom;
        bool running;
    public:
        ROSThread(ros::NodeHandle nh);
        ~ROSThread();
        void onLaserScan(sensor_msgs::LaserScan scan);
        void onNavMsg(nav_msgs::Odometry odometry);
    signals:
        void scanRecieved(const QPixmap& map, int x, int y, float theta);
};
#endif

