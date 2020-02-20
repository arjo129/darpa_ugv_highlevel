#include <mission_planner/RosThread.h>

ROSThread::ROSThread(ros::NodeHandle parentNh, int robotNum): nh(parentNh, ROBOT_NAME(robotNum)) 
{
    scanStampedSub = nh.subscribe(ROBOT_SCAN_TOPIC(robotNum), 10, &ROSThread::onLaserScanStampedCb, this);
    co2Sub = nh.subscribe(ROBOT_CO2_TOPIC(robotNum), 10, &ROSThread::onCo2Cb, this);
    wifiSignalSub = nh.subscribe(ROBOT_WIFI_TOPIC(robotNum), 10, &ROSThread::onWifiSignalCb, this);
    this->robotStartPub = nh.advertise<std_msgs::String>(ROBOT_START_TOPIC(robotNum), 1);
    this->robotEStopPub = nh.advertise<std_msgs::String>(ROBOT_ESTOP_TOPIC(robotNum), 1);
    this->robotGoalPub = nh.advertise<geometry_msgs::Pose>(ROBOT_GOAL_TOPIC(robotNum), 1);
    this->robotLoraDropPub = nh.advertise<std_msgs::String>(ROBOT_DROP_TOPIC(robotNum), 1);
    this->robotNum = robotNum;
    this->running = true;
    this->numLoraDropped = 0;
}

ROSThread::~ROSThread() 
{
    this->running = false;
}

void ROSThread::onLaserScanStampedCb(data_compresor::ScanStamped scanStamped)
{
    onNavMsg(scanStamped.odom);
    onLaserScan(scanStamped.scan);
}

void ROSThread::onCo2Cb(wireless_msgs::Co2 co2) 
{
    std::string details = "Seq No: " + std::to_string(co2.header.seq) + \
                                ", C02-Conc: " + std::to_string(co2.concentration);
    emit artifactReceived(co2.position.x, co2.position.y, co2.position.z, details);
    ROS_INFO("Received CO2 signal data");
}

void ROSThread::onWifiSignalCb(wireless_msgs::WifiArray wifiReport)
{
    for (int idx = 0; idx < wifiReport.data.size(); idx++) {
        wireless_msgs::Wifi wifi = wifiReport.data[idx];
        std::string details = "Seq No: " + std::to_string(wifiReport.header.seq);
        details += ", SSID: " + std::string(wifi.ssid.data.c_str()) + \
                            ", Signal: " + std::string(wifi.signal.data.c_str());
        emit artifactReceived(wifiReport.position.x, wifiReport.position.y, wifiReport.position.z, details);
    }

    ROS_INFO("Received wifi signal data");
}

void ROSThread::onLaserScan(sensor_msgs::LaserScan lscan) 
{
    const int size = 500;
    float angle = lscan.angle_min;
    QImage* image = new QImage(size, size, QImage::Format::Format_RGB888);
    image->fill(QColor(Qt::white).rgb());
    for(int i = 0; i < lscan.ranges.size(); i++ ){
        if(lscan.ranges[i] > lscan.range_max) {
             angle+=lscan.angle_increment;
             continue;
        }
        int x = round(size/2*(lscan.ranges[i]/10)*cos(angle))+size/2;
        int y = round(size/2*(lscan.ranges[i]/10)*sin(angle))+size/2;
        if(x > size || y > size || x < 0|| y < 0){
            angle+=lscan.angle_increment;
            continue;
        }
        image->setPixelColor(x,y,QColor(255,0,0));
        angle+=lscan.angle_increment;
    }
    float x = 25*this->recentOdom.pose.pose.position.x;
    float y = 25*this->recentOdom.pose.pose.position.y;
    this->recentOdom.pose.pose.orientation;

    tf::Quaternion q(
        this->recentOdom.pose.pose.orientation.x,
        this->recentOdom.pose.pose.orientation.y,
        this->recentOdom.pose.pose.orientation.z,
        this->recentOdom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    QPixmap pixmap = QBitmap::fromImage(*image);
    emit scanRecieved(robotNum, pixmap, x, y, yaw);
}

void ROSThread::onNavMsg(nav_msgs::Odometry odom) 
{
    this->recentOdom = odom;
}

// cancels the e-stop operation to make robot autonomous again
void ROSThread::startRobot() 
{
    std_msgs::String msg;
    msg.data = "start"; // any non-empty string will work
    robotStartPub.publish(msg);
    ROS_INFO("Cancelling E-Stop and starting Robot number %d", this->robotNum);
}

// stop the robot immediately
void ROSThread::eStopRobot() 
{
    std_msgs::String msg;
    msg.data = "estop"; // any non-empty string will work
    robotEStopPub.publish(msg);
    ROS_INFO("E-Stopping Robot number %d", this->robotNum);
}

void ROSThread::sendRobotGoal(double x, double y, double theta) 
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;

    tf2::Quaternion quatTF;
    double yawRad = theta * M_PI / 180.0;
    quatTF.setRPY(0, 0, yawRad);
    geometry_msgs::Quaternion quatMsg = tf2::toMsg(quatTF);
    pose.orientation = quatMsg;

    this->robotGoalPub.publish(pose);
}

void ROSThread::dropLoraNode() 
{
    std_msgs::String stringMsg;
    stringMsg.data = "Dropping LORA";
    this->robotLoraDropPub.publish(stringMsg);
    this->numLoraDropped++;
    float x = this->recentOdom.pose.pose.position.x;
    float y = this->recentOdom.pose.pose.position.y;
    float z = this->recentOdom.pose.pose.position.z;
    ROS_INFO("Robot_%d: Dropping LORA number %d at (%f, %f, %f)", this->robotNum, this->numLoraDropped, x, y, z);
}

void ROSThread::run() 
{
    ros::Rate rate(10);
    while(this->running) {
        ros::spinOnce();
        rate.sleep();
    }
}