#include <mission_planner/RosThread.h>

ROSThread::ROSThread(ros::NodeHandle parentNh, int robotNum): nh(parentNh, ROBOT_NAME(robotNum)) 
{
    scanStampedSub = nh.subscribe(ROBOT_SCAN_TOPIC(robotNum), 10, &ROSThread::onLaserScanStamped, this);
    robotStartPub = nh.advertise<std_msgs::String>(ROBOT_START_TOPIC(robotNum), 1);
    robotEStopPub = nh.advertise<std_msgs::String>(ROBOT_ESTOP_TOPIC(robotNum), 1);
    this->robotNum = robotNum;
    this->running = true;
}

ROSThread::~ROSThread() 
{
    this->running = false;
}

void ROSThread::onLaserScanStamped(data_compresor::ScanStamped scanStamped)
{
    onNavMsg(scanStamped.odom);
    onLaserScan(scanStamped.scan);
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

void ROSThread::run() 
{
    ros::Rate rate(10);
    while(this->running) {
        ros::spinOnce();
        rate.sleep();
    }
}