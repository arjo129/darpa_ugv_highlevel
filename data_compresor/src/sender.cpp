/**
 * This implements the telemetry sent by the UGV 
 */ 
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include <vehicle_drive/Dropper.h>
#include <wireless_msgs/LoraPacket.h>
#include <data_compressor/msgs/laserscan.h>
#include <data_compressor/msgs/co2.h>
#include <data_compressor/msgs/WifiArray.h>
#include <data_compressor/msgs/goal.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* moveBaseClient;

class CompressedTelemetrySender {

    ros::Publisher loraPub, estopPub, startPub, dropperPub;
    ros::Subscriber laserscan, co2, wifi, loraSubscriber, odomSub;
    nav_msgs::Odometry lastOdom;
    bool recieved_odom = false;
    tf::TransformListener* tfListener;
    wireless_msgs::LoraPacket scanPacket; 
    
    void compressScan(sensor_msgs::LaserScan scan) {
        static int count = 0;
        if(!recieved_odom){
            return;
        }
        std::vector<AdaptiveTelemetryScan> result = convertLaserScan(scan, lastOdom, 4);
        scanPacket = toLoraPacket(result[0]);
        scanPacket.to.data = "base_station";
        if(count%50 ==0)
            loraPub.publish(scanPacket);
        count++;
    }

    void onRecieveOdom(nav_msgs::Odometry odom) {
        this->lastOdom = odom;
        this->recieved_odom = true;
        //std::cout << "recieved odom" << odom <<std::endl;
    }

    void onRecieveLora(wireless_msgs::LoraPacket packet) {
        static int dropper_states[3];
        static int dropper_index = 0;
        static bool first = true;
        if(first){
            first =false;
            dropper_states[0] = 60;
            dropper_states[1] = 60;
            dropper_states[2] = 60;
        }
        std::vector<uint8_t> data = uncompressZip(packet.data);
        if(data.size() < 0){
            ROS_ERROR("Failed to decompress packet");
            return;
        }
        if(data[0] == (uint8_t) MessageType::ESTOP){
            std_msgs::String str;
            str.data = "LoRA E-Stop";
            estopPub.publish(str);
            std::vector<uint8_t> response;
            wireless_msgs::LoraPacket respPacket;
            respPacket.to = packet.from;
            response.push_back((uint8_t)MessageType::ESTOP_ACK);
            respPacket.data = compressZip(response);
            loraPub.publish(respPacket);
            ROS_INFO("Stopping due to E-Stop signal");
            return;
        }
        if(data[0] == (uint8_t) MessageType::START){
            std_msgs::String str;
            str.data = "hi";
            startPub.publish(str);
            return;
        }
        if(data[0] == (uint8_t) MessageType::GOTO_GOAL){
            Goal target = decodeGoal(data);

            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.pose.position.x = (float)target.x/100;
            goal.target_pose.pose.position.y = (float)target.y/100;
            goal.target_pose.pose.position.z = 0;
            float targetYaw = atan2(target.y, target.x);
            tf::Quaternion targetYawQt(tf::Vector3(0,0,1), targetYaw);
            goal.target_pose.pose.orientation.x = targetYawQt.x();
            goal.target_pose.pose.orientation.y = targetYawQt.y();
            goal.target_pose.pose.orientation.z = targetYawQt.z();
            goal.target_pose.pose.orientation.w = targetYawQt.w();
            
            moveBaseClient->sendGoal(goal);
            std::cout << "recieved goal" <<std::endl;
            
            wireless_msgs::LoraPacket respPacket;
            std::vector<uint8_t> response;
            respPacket.to = packet.from;
            response.push_back((uint8_t)MessageType::GOAL_ACK);
            respPacket.data = compressZip(response);
            loraPub.publish(respPacket);

            std_msgs::String str;
            str.data = "hi";
            startPub.publish(str);
        }

        if(data[0] == (uint8_t) MessageType::DROP_NODE) {
            vehicle_drive::Dropper dropmsg;
            if(dropper_index>2) return;
            dropper_states[dropper_index] = 0;
            dropmsg.dropper_angles[0] = dropper_states[0];
            dropmsg.dropper_angles[1] = dropper_states[1];
            dropmsg.dropper_angles[2] = dropper_states[2];
            dropperPub.publish(dropmsg);
            dropper_index++;
        }
    }

    void onCO2Reading(std_msgs::UInt16 reading){
        //if(reading.data > 1800){
            Co2 co2;
            co2.concentration = reading.data;
            co2.x = lastOdom.pose.pose.position.x;
            co2.y = lastOdom.pose.pose.position.y;
            co2.z = lastOdom.pose.pose.position.z;
            co2.timestamp = ros::Time::now().toNSec();
            wireless_msgs::LoraPacket packet = toLoraPacket(co2);
            packet.to.data = "base_station";
            loraPub.publish(packet);
        //}
    }

    void onWifiScan(wireless_msgs::WifiArray wifi){
        wireless_msgs::LoraPacket packet = toLoraPacket(wifi);
        loraPub.publish(packet);
    }

public:
    CompressedTelemetrySender(ros::NodeHandle& nh) {
        //Laser scan compressor
        
        this->loraPub = nh.advertise<wireless_msgs::LoraPacket>("/lora/tx", 10);
        this->estopPub = nh.advertise<std_msgs::String>("/e_stop", 10);
        this->dropperPub = nh.advertise<vehicle_drive::Dropper>("droppers",10);
        this->tfListener = new tf::TransformListener();
        this->laserscan = nh.subscribe("/scan", 10, &CompressedTelemetrySender::compressScan, this);
        this->odomSub = nh.subscribe("/odom", 10, &CompressedTelemetrySender::onRecieveOdom, this);
        this->co2 = nh.subscribe("/co2", 10, &CompressedTelemetrySender::onCO2Reading, this);
        this->wifi = nh.subscribe("/wifi", 10, &CompressedTelemetrySender::onWifiScan, this);
        this->loraSubscriber = nh.subscribe("/lora/rx", 10, &CompressedTelemetrySender::onRecieveLora, this); 
        this->startPub = nh.advertise<std_msgs::String>("/start", 10); 
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "compression_node");
    ros::NodeHandle nh;
    CompressedTelemetrySender telemetry(nh);
    moveBaseClient = new MoveBaseClient("move_base", true);
    while(!moveBaseClient->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ros::Rate rate(1.5);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
