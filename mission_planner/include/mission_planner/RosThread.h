#ifndef _ROS_THREAD_H_
#define _ROS_THREAD_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <data_compresor/ScanStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <QThread>
#include <QBitmap> 
#include <QImage>
#include <mission_planner/Config.h>
#include <wireless_msgs/Co2.h>
#include <wireless_msgs/WifiArray.h>

inline std::string stringConcat(const std::string& a, const std::string& b)
{
    return a + b;
}

#define ROBOT_NAME(x) stringConcat("/robot_", std::to_string(x))

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
    #define ROBOT_ESTOP_TOPIC(x) "/e_stop"
    #define ROBOT_START_TOPIC(x) "/start"
    #define ROBOT_GOAL_TOPIC(x) "/goal"
    #define ROBOT_DROP_TOPIC(x) "/dropper"
    

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
    #define ROBOT_ESTOP_TOPIC(x) stringConcat(ROBOT_NAME(x), "/e_stop")
    #define ROBOT_START_TOPIC(x) stringConcat(ROBOT_NAME(x), "/start")
    #define ROBOT_GOAL_TOPIC(x) stringConcat(ROBOT_NAME(x), "/goal")
    #define ROBOT_DROP_TOPIC(x) stringConcat(ROBOT_NAME(x), "/dropper")
    #define ROBOT_AUTONOMY_STATE_TOPIC(x) stringConcat(ROBOT_NAME(x), "/autonomy_state")


#endif


class ROSThread: public QThread {

    Q_OBJECT
    protected:
        void run() override;
    private:
        ros::NodeHandle nh;
        ros::Subscriber scanStampedSub;
        ros::Subscriber co2Sub;
        ros::Subscriber wifiSignalSub;
        ros::Publisher robotStartPub; // cancels E-stop
        ros::Publisher robotEStopPub;
        ros::Publisher robotGoalPub;
        ros::Publisher robotLoraDropPub;
        bool running;
        int robotNum;
        int numLoraDropped;
        nav_msgs::Odometry recentOdom; 
    public:
        ROSThread(ros::NodeHandle parentNh, int robotNum);
        ROSThread(const ROSThread& old_obj);
        ~ROSThread();
        void onLaserScanStampedCb(data_compresor::ScanStamped scanStamped);
        void onCo2Cb(wireless_msgs::Co2);
        void onWifiSignalCb(wireless_msgs::WifiArray);
        void onLaserScan(sensor_msgs::LaserScan);
        void onNavMsg(nav_msgs::Odometry odometry);
        void start();
        void stop();
        void sendGoal(double x, double y, double theta);
        void dropLora();
    signals:
        void scanRecieved(int robotNum, const QPixmap& map, float x, float y, float theta);
        void artifactReceived(float x, float y, float z, std::string details);
};
#endif

