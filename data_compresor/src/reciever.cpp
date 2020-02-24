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
#include <data_compressor/msgs/goal.h>
#include <data_compresor/ScanStamped.h>
/*Global cause who gives a damn about coding standards 1 week before a competition*/
ros::NodeHandle *nh;
ros::Subscriber loraSub;
ros::Publisher loraPub;
/**
 * Robot state 
`*/
enum class RobotState {
    ESTOP_IN_PROG, ESTOPPED, OK
};
/**
 * Goal State
 */ 
enum class GoalState {
    GOAL_INPROG, NO_GOAL
};

/**
 * E-Stop State
 */ 
enum class AutonomyState{
    AUTONOMOUS_PENDING, AUTONOMOUS, TELEOP_PENDING, TELEOP
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
    ros::Subscriber dropper;
    ros::Subscriber autonomySub;

    std::string robot_name;

    GoalState goalstate;
    wireless_msgs::LoraPacket lastGoal;
    RobotState state;
    AutonomyState autonomystate;
 public:
    Robot(){}//Hack to silence compiler errors.
    Robot(std::string name): robot_name(name) {
        laserPub = nh->advertise<data_compresor::ScanStamped>(robot_name+"/scan", 10);
        co2Pub = nh->advertise<wireless_msgs::Co2>(robot_name+"/co2", 10);
        waPub = nh->advertise<wireless_msgs::WifiArray>(robot_name+"/wifi", 10);
        odomPub = nh->advertise<nav_msgs::Odometry>(robot_name+"/odom", 10);

        estop = nh->subscribe(robot_name+"/e_stop", 10, &Robot::onRecieveEstop, this);
        startSub = nh->subscribe(robot_name+"/start", 10, &Robot::onRecieveStart, this);
        robotGoal = nh->subscribe(robot_name+"/goal", 10, &Robot::onRecieveGoal, this);
        dropper = nh->subscribe(robot_name+"/dropper", 10, &Robot::dropNode, this);
        autonomySub = nh->subscribe(robot_name+"/autonomy_state", 10, &Robot::autonomyState, this);
        this->state = RobotState::OK;
        goalstate = GoalState::NO_GOAL;
        autonomystate = AutonomyState::TELEOP;
    }

    void autonomyState(std_msgs::String state) {
        if(state.data=="autonomous") {
            this->autonomystate = AutonomyState::AUTONOMOUS_PENDING;
        } else {
            this->autonomystate = AutonomyState::TELEOP_PENDING;
        }
    }

    void dropNode(std_msgs::String  str) {
        wireless_msgs::LoraPacket packet;
        packet.to.data = robot_name;
        std::vector<uint8_t> signal;
        signal.push_back((uint8_t)MessageType::DROP_NODE);
        packet.data  = compressZip(signal);
        loraPub.publish(packet);
    }

    void publish(data_compresor::ScanStamped scan){
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

    void onRecieveStart(std_msgs::String str) {
        start();
    }

    void onRecieveGoal(geometry_msgs::Pose pose){
        Goal goal;
        goal.x = pose.position.x*100;
        goal.y = pose.position.y*100;
        wireless_msgs::LoraPacket packet = toLoraPacket(goal);
        packet.to.data = robot_name;
        lastGoal = packet;
        loraPub.publish(packet);
        this->goalstate = GoalState::GOAL_INPROG;
    }

    void EStop() {
        this->state = RobotState::ESTOP_IN_PROG;
    }

    void EStopAck() {
        this->state = RobotState::ESTOPPED;
    }

    void goalAck() {
        this->goalstate = GoalState::NO_GOAL;
    }

    void autonomousAck() {
        this->autonomystate = AutonomyState::AUTONOMOUS;
    }

    void teleopAck() {
        this->autonomystate = AutonomyState::TELEOP;
    }

    void start(){
        wireless_msgs::LoraPacket packet;
        packet.to.data = robot_name;
        std::vector<uint8_t> signal;
        signal.push_back((uint8_t)MessageType::START);
        packet.data  = compressZip(signal);
        loraPub.publish(packet);
    }

    std::vector<wireless_msgs::LoraPacket> flushLoraOut(){
        std::vector<wireless_msgs::LoraPacket> packets;
        if(this->state == RobotState::ESTOP_IN_PROG){
            wireless_msgs::LoraPacket packet;
            packet.to.data = robot_name;
            std::vector<uint8_t> signal;
            signal.push_back((uint8_t)MessageType::ESTOP);
            packet.data  = compressZip(signal);
            packets.push_back(packet);
            
        }
        if(this->goalstate == GoalState::GOAL_INPROG){
            packets.push_back(lastGoal);
        }
        if(this->autonomystate == AutonomyState::AUTONOMOUS_PENDING) {
            wireless_msgs::LoraPacket packet;
            packet.to.data = robot_name;
            std::vector<uint8_t> signal;
            signal.push_back((uint8_t)MessageType::AUTONOMOUS_NOW);
            packet.data  = compressZip(signal);
            packets.push_back(packet);
        }
        if(this->autonomystate == AutonomyState::TELEOP_PENDING) {
            wireless_msgs::LoraPacket packet;
            packet.to.data = robot_name;
            std::vector<uint8_t> signal;
            signal.push_back((uint8_t)MessageType::TELEOP_NOW);
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
std::unordered_map<std::string, Robot*> robot_list;

/**
 * Get the robot by name
 */ 
Robot* lookupOrCreateRobot(std::string robot){
    if(robot_list.count(robot) == 0){
        robot_list[robot] = new Robot(robot);
    }
    return robot_list[robot];
}

void initRobots(int numRobots) {
    for (int idx = 1; idx <= numRobots; idx++) {
        std::string robotName = "robot_" + std::to_string(idx);
        lookupOrCreateRobot(robotName);
    }
}

void handleLaserScan(std::string from, std::vector<uint8_t> data) {
    AdaptiveTelemetryScan scan = decodeScan(data);
    sensor_msgs::LaserScan lscan = toLaserScan(scan);
    data_compresor::ScanStamped stampedScan;
    stampedScan.odom = getOdom(scan);
    stampedScan.scan = lscan;
    lookupOrCreateRobot(from)->publish(stampedScan);
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

void handleGoalAck(std::string from, std::vector<uint8_t> data){
    lookupOrCreateRobot(from)->goalAck();
}

void handleAutonomousAck(std::string from, std::vector<uint8_t> data){
    lookupOrCreateRobot(from)->autonomousAck();
}

void handleTeleopAck(std::string from, std::vector<uint8_t> data){
    lookupOrCreateRobot(from)->teleopAck();
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
        case (uint8_t)MessageType::GOAL_ACK:
            handleGoalAck(packet.from.data, data);
            break;
        case (uint8_t)MessageType::AUTONOMOUS_NOW:
            handleAutonomousAck(packet.from.data, data);
            break;
        case (uint8_t)MessageType::TELEOP_NOW:
            handleGoalAck(packet.from.data, data);
            break;
        default:
            ROS_ERROR("Handler not found");
    }
}

void flushAllRobotsBuffers() {
    for(auto robots : robot_list){
        Robot* robot = robots.second;
        std::vector<wireless_msgs::LoraPacket> packets= robot->flushLoraOut();
        for(wireless_msgs::LoraPacket packet: packets) {
            loraPub.publish(packet);
        }
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "basestation_relay");
    nh = new ros::NodeHandle();
    ros::Rate rate(0.3);
    loraSub = nh->subscribe("/lora/rx", 10, &onRecieveRx);
    loraPub = nh->advertise<wireless_msgs::LoraPacket>("/lora/tx", 10);
    initRobots(5);
    while(ros::ok()){
        ros::spinOnce();
        flushAllRobotsBuffers();
        rate.sleep();
    }
    delete nh;
    return 0;
}
