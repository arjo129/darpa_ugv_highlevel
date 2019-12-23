#include <ros/ros.h>
#include <wireless_msgs/uwb.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <unordered_map>
#include <random>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>

class UWBTracker {
    std::vector<Eigen::Vector3f> particles;
    std::vector<float> weights;
    ros::Time lastSeen;
    Eigen::Vector3f prevPosition;
    std::string frame_id;
    std::default_random_engine generator;
    int numParticles;
    float sigma;
public:
    UWBTracker() {
        this->prevPosition = Eigen::Vector3f(0,0,0);
        this->numParticles = 50000;
        sigma = 0.4;
    }
    void applyMotionModel(nav_msgs::Odometry odom) {
        Eigen::Vector3f currentPose(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        Eigen::Vector3f movement = - currentPose - this->prevPosition;
        float x_stdev = odom.pose.covariance[0];
        float y_stdev = odom.pose.covariance[7];
        float z_stdev = odom.pose.covariance[14];
        std::normal_distribution<double> x_noise(0, x_stdev);
        std::normal_distribution<double> y_noise(0, y_stdev);
        std::normal_distribution<double> z_noise(0, z_stdev);
        this->frame_id = odom.header.frame_id;
        for(int i = 0; i < this->particles.size(); i++) {
            Eigen::Vector3f noise(x_noise(generator), y_noise(generator), z_noise(generator));
            this->particles[i] += movement + noise;
        }
        this->prevPosition = currentPose;
    }

    void spawnParticles(nav_msgs::Odometry odom, float distance) {
        Eigen::Vector3f currentPose(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        this->prevPosition = currentPose;
        this->frame_id = odom.header.frame_id;

        std::normal_distribution<double> normalDistribution(distance, this->sigma);
        std::uniform_real_distribution<double> uniform(-M_PI, M_PI);

        for(int i  = 0; i < this->numParticles; i++) {
            double r = normalDistribution(generator);
            double theta = uniform(generator);
            double phi = uniform(generator);
            Eigen::Vector3f particle(r*sin(theta)*cos(phi), r*sin(theta)*sin(phi), r*cos(theta));
            this->particles.push_back(particle);
            this->weights.push_back(1.0f/this->numParticles);
        }
    }
    void updateMeasurement(float distance){
        for(int i  = 0; i < this->numParticles; i++) {
            float x = (this->particles[i] - prevPosition).norm();
            float z = (x-distance)/(1.414213*this->sigma);
            float res = exp(-(z*z));
            float scalingFactor = 1/sqrt(2*M_PI*this->sigma*this->sigma);
            res *= scalingFactor;
            this->weights[i] *= res; 
        }
    }
    
    void resample() {
        float sum = 0;
        for(int i  = 0; i < this->numParticles; i++) {
            sum += this->weights[i];
        }
        
        std::uniform_real_distribution<float> uniform(0, sum/this->numParticles);
        float r = uniform(generator);
        float accumulator = this->weights[0];
        int weight_index = 0;
        std::vector<Eigen::Vector3f> resampled_particles;
        std::vector<float> resampled_weights;
        for(int i  = 0; i < this->numParticles; i++) {
            float u = r + sum*(i-1)/this->numParticles;
            while(u > accumulator){
                weight_index++;
                accumulator += this->weights[weight_index%this->numParticles];
            }
            resampled_particles.push_back(this->particles[weight_index%this->numParticles]);
            resampled_weights.push_back(this->weights[weight_index%this->numParticles]);
        }
        
        this->particles.clear();
        this->weights.clear();
        for(int i  = 0; i < this->numParticles; i++) {
            this->particles.push_back(resampled_particles[i]);
            this->weights.push_back(resampled_weights[i]);
        }

    }

    visualization_msgs::Marker visuallizeParticles(){

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.id =0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1;
        for(int i = 0; i < particles.size(); i+=100) {
            geometry_msgs::Point pt;
            pt.x = this->particles[i].x();
            pt.y = this->particles[i].y();
            pt.z = this->particles[i].z();
            marker.points.push_back(pt);
        }
        return marker;    
    }
};
class LocalizationManager {
    ros::NodeHandle nh;
    ros::Subscriber uwb, odometry, commands, imu;
    ros::Publisher pub;
    bool odomRecieved = false;
    nav_msgs::Odometry currentOdom;
    
    std::unordered_map<std::string, UWBTracker> uwb_trackers;
    
    void onUWBRecieve(wireless_msgs::uwb uwb_msg) {
        if(!this->odomRecieved) {
            return;
        }
        if(this->uwb_trackers.count(uwb_msg.name.data) == 0) {
            this->uwb_trackers[uwb_msg.name.data] = UWBTracker();
            this->uwb_trackers[uwb_msg.name.data].spawnParticles(currentOdom, uwb_msg.distance.data);
            return;
        }
        this->uwb_trackers[uwb_msg.name.data].applyMotionModel(this->currentOdom);       
        this->uwb_trackers[uwb_msg.name.data].updateMeasurement(uwb_msg.distance.data);
        this->uwb_trackers[uwb_msg.name.data].resample();
    }

    void onOdometryRecieve(nav_msgs::Odometry odom) {
        this->currentOdom = odom;
        odomRecieved = true;
    }

    void onCommandRecieve(geometry_msgs::Twist twist) {
        
    }

    void onImuRecieve(sensor_msgs::Imu imu_reading) {

    }

    

public:
    void spin() {
        visualization_msgs::MarkerArray markers;
        for(auto it: this->uwb_trackers) {
            markers.markers.push_back(it.second.visuallizeParticles());
        }
        pub.publish(markers);
    }
    LocalizationManager(ros::NodeHandle _nh){
        this->nh = _nh;
        uwb = this->nh.subscribe("/husky1/uwb_rangers", 10, &LocalizationManager::onUWBRecieve, this);
        odometry = this->nh.subscribe("/husky1/odom_rf2o", 10, &LocalizationManager::onOdometryRecieve, this);
        commands = this->nh.subscribe("/husky1/husky_velocity_controller/cmd_vel", 10, &LocalizationManager::onCommandRecieve, this);
        imu =  this->nh.subscribe("/husky1/imu/data", 10, &LocalizationManager::onImuRecieve, this);
        pub = this->nh.advertise<visualization_msgs::MarkerArray>("/husky1/uwb/markers", 10);
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "localization_manager");
    ros::NodeHandle nh;
    LocalizationManager lm(nh);
    ros::Rate r(10);
    while(ros::ok()) {
        ros::spinOnce();
        lm.spin();
        r.sleep();
    }
}