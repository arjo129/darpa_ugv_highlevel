/***
 * Decodes packets coming in via LoRA and sends heartbeat status signal to all robots
 * 
 */  
#include <unordered_map>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <wireless_msgs/LoraPacket.h> 
#include <wifi_sensor/wifiArray.h>
#include <data_compressor/zip.h>
#include <data_compressor/protocol.h>
#include <data_compressor/msgs/laserscan.h>
#include <data_compressor/msgs/co2.h>
#include <data_compressor/msgs/WifiArray.h>

/*Global cause who gives a damn about coding standards 1 week before a competition*/
ros::NodeHandle *nh;
ros::Subscriber loraSub;
ros::Publisher loraPub;
/**
 * Robot state 
`*/
enum class RobotState {
    ESTOP_IN_PROG, ESTOPPED
};
/**
 * Define an individual robot
 */ 
class Robot {
    ros::Publisher laserPub;
    ros::Publisher co2Pub;
    ros::Publisher waPub;
    ros::Publisher odomPub;

    ros::Subscriber robotGoal;
    ros::Subscriber estop;
    ros::Subscriber startSub;

    std::string robot_name;

    RobotState state;
 public:
    Robot(){}//Hack to silence compiler errors.
    Robot(std::string name): robot_name(name) {
        laserPub = nh->advertise<sensor_msgs::LaserScan>(robot_name+"/scan", 10);
        co2Pub = nh->advertise<wireless_msgs::Co2>(robot_name+"/co2", 10);
        waPub = nh->advertise<wireless_msgs::WifiArray>(robot_name+"/wifi", 10);
        odomPub = nh->advertise<nav_msgs::Odometry>(robot_name+"/odom", 10);

        estop = nh->subscribe(robot_name+"/e_stop", 10, &Robot::onRecieveEstop, this);
    }

    void publish(sensor_msgs::LaserScan scan){
        laserPub.publish(scan);
    }

    void publish(nav_msgs::Odometry odom) {
        odomPub.publish(odom);
    }

    void publish(wireless_msgs::Co2 co2) {
        co2Pub.publish(co2);
    }

    void publish(wireless_msgs::WifiArray wifi) {
        waPub.publish(wifi);
    }

    void onRecieveEstop(std_msgs::String str) {
        EStop();
    }

    void EStop() {
        state = RobotState::ESTOP_IN_PROG;
    }

    void EStopAck() {
        state = RobotState::ESTOPPED;
    }

    void start(){

    }

    std::vector<wireless_msgs::LoraPacket> flushLoraOut(){
        std::vector<wireless_msgs::LoraPacket> packets;
        if(state == RobotState::ESTOP_IN_PROG){
            wireless_msgs::LoraPacket packet;
            packet.to.data = robot_name;
            std::vector<uint8_t> signal;
            signal.push_back((uint8_t)MessageType::ESTOP);
            packet.data  = compressZip(signal);
            packets.push_back(packet);
            
        }
        return packets;
    }

    std::string getName() const {
        return robot_name;
    }

    bool operator==(const Robot & other) const {
        return this->robot_name == other.getName();
    }
};

namespace std{
  template<>
    struct hash<Robot>
    {
      size_t
      operator()(const Robot & obj) const
      {
        return hash<std::string>()(obj.getName());
      }
    };
};

/**
 * Maintain a list of robots
 */ 
std::unordered_map<std::string, Robot> robot_list;

/**
 * Get the robot by name
 */ 
Robot* lookupOrCreateRobot(std::string robot){
    if(robot_list.count(robot) == 0){
        robot_list[robot] = Robot(robot);
    }
    return &robot_list[robot];
}

void handleLaserScan(std::string from, std::vector<uint8_t> data) {
    AdaptiveTelemetryScan scan = decodeScan(data);
    sensor_msgs::LaserScan lscan = toLaserScan(scan);
    lookupOrCreateRobot(from)->publish(getOdom(scan));
    lookupOrCreateRobot(from)->publish(lscan);
}

void handleCo2(std::string from, std::vector<uint8_t> data) {
    Co2 co2 = decodeCo2(data);
    wireless_msgs::Co2 msg = toCo2(co2);
    lookupOrCreateRobot(from)->publish(msg);
}

void handleWifiArray(std::string from, std::vector<uint8_t> data) {
    WifiArray wa = decodeWifiArray(data);
    wireless_msgs::WifiArray msg = toWifiArray(wa);
    lookupOrCreateRobot(from)->publish(msg);
}

void handleEStopAck(std::string from, std::vector<uint8_t> data) {
    lookupOrCreateRobot(from)->EStopAck();
}

/**
 * Routes the packet to the correct decompressor
 */ 
void onRecieveRx(wireless_msgs::LoraPacket packet) {
    std::vector<uint8_t> data = uncompressZip(packet.data);
    if(data.size() < 0){
        ROS_ERROR("Failed to decompress packet");
        return;
    }
    switch(data[0]) {
        case (uint8_t)MessageType::LASER_SCAN:
            handleLaserScan(packet.from.data, data);
            break;
        case (uint8_t)MessageType::ESTOP_ACK:
            handleEStopAck(packet.from.data, data);
        case (uint8_t)MessageType::CO2_SIGNATURE:
            handleCo2(packet.from.data, data);
            break;
        case (uint8_t)MessageType::WIFI_SIGNAL:
            handleWifiArray(packet.from.data, data);
            break;
        default:
            ROS_ERROR("Handler not found");
    }
}

void flushAllRobotsBuffers() {
    for(auto robots : robot_list){
        Robot robot = robots.second;
        std::vector<wireless_msgs::LoraPacket> packets= robot.flushLoraOut();
        for(wireless_msgs::LoraPacket packet: packets) {
            loraPub.publish(packet);
        }
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "basestation_relay");
    nh = new ros::NodeHandle();
    ros::Rate rate(10);
    loraSub = nh->subscribe("/lora/rx", 10, &onRecieveRx);
    loraPub = nh->advertise<wireless_msgs::LoraPacket>("/lora/tx", 10);
    while(ros::ok()){
        ros::spinOnce();
        flushAllRobotsBuffers();
        rate.sleep();
    }
    delete nh;
    return 0;
}
