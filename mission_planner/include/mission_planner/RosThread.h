#ifndef _ROS_THREAD_H_
#define _ROS_THREAD_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <QThread>
#include <QBitmap> 
#include <QImage>
#include <mission_planner/Config.h>

inline std::string stringConcat(const std::string& a, const std::string& b)
{
    return a + b;
}

#define ROBOT_NAME(x) stringConcat("/robot", std::to_string(x))

// Definitions for single robot debugging over ROS network
#ifdef SINGLE_ROBOT_DEBUG

    #define ROBOT_SCAN_TOPIC(x) "/scan"
    #define ROBOT_WIFI_TOPIC(x) "/wifi"
    #define ROBOT_CO2_TOPIC(x)  "/co2"
    #define ROBOT_VENTS_TOPIC(x) "/vents"
    #define ROBOT_MANAKIN_TOPIC(x) "/manakin"
    #define ROBOT_STATUS_TOPIC(x) "/status"
    #define ROBOT_POOP_TRAIL_TOPIC(x) "/poop_trail"
    #define ROBOT_ODOM_TOPIC(x) "/odom_rf2o" 
    #define ROBOT_ESTOP_TOPIC(x) "/estop"
    #define ROBOT_START_TOPIC(x) "/start"
    

#endif

// Normal multi-robot definitions over LoRa network
#ifndef SINGLE_ROBOT_DEBUG

    #define ROBOT_SCAN_TOPIC(x) stringConcat(ROBOT_NAME(x), "/scan")
    #define ROBOT_WIFI_TOPIC(x) stringConcat(ROBOT_NAME(x), "/wifi")
    #define ROBOT_CO2_TOPIC(x) stringConcat(ROBOT_NAME(x), "/co2")
    #define ROBOT_VENTS_TOPIC(x) stringConcat(ROBOT_NAME(x), "/vents")
    #define ROBOT_MANAKIN_TOPIC(x) stringConcat(ROBOT_NAME(x), "/manakin")
    #define ROBOT_STATUS_TOPIC(x) stringConcat(ROBOT_NAME(x), "/status")
    #define ROBOT_POOP_TRAIL_TOPIC(x) stringConcat(ROBOT_NAME(x), "/poop_trail")
    #define ROBOT_ODOM_TOPIC(x) stringConcat(ROBOT_NAME(x), "/odom") 
    #define ROBOT_ESTOP_TOPIC(x) stringConcat(ROBOT_NAME(x), "/estop")
    #define ROBOT_START_TOPIC(x) stringConcat(ROBOT_NAME(x), "/start")

#endif


class ROSThread: public QThread {

    Q_OBJECT
    protected:
        void run() override;
    private:
        ros::NodeHandle nh;
        ros::Subscriber laserScanSub;
        ros::Subscriber odometrySub;
        ros::Publisher robotStartPub; // cancels E-stop
        ros::Publisher robotEStopPub;
        nav_msgs::Odometry recentOdom;
        bool running;
        uint8_t robotNum;
    public:
        ROSThread(ros::NodeHandle parentNh, uint8_t robotNum);
        ~ROSThread();
        void onLaserScan(sensor_msgs::LaserScan scan);
        void onNavMsg(nav_msgs::Odometry odometry);
        void startRobot();
        void eStopRobot();
    signals:
        void scanRecieved(uint8_t robotNum, const QPixmap& map, int x, int y, float theta);
};
#endif

