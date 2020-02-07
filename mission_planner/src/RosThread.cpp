#include <mission_planner/RosThread.h>
#include <QBitmap> 
#include <QImage>
#include <tf/tf.h>

ROSThread::ROSThread(ros::NodeHandle parentNh, uint8_t robotNum): nh(parentNh, ROBOT_NAME(robotNum)) 
{
    laserScanSub = nh.subscribe(ROBOT_SCAN_TOPIC(robotNum), 10, &ROSThread::onLaserScan, this);
    odometrySub = nh.subscribe(ROBOT_ODOM_TOPIC(robotNum),10,  &ROSThread::onNavMsg, this);
    this->running = true;
}

ROSThread::~ROSThread() 
{
    this->running = false;
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
    emit scanRecieved(pixmap, x, y, yaw);
}

void ROSThread::onNavMsg(nav_msgs::Odometry odom) 
{
    this->recentOdom = odom;
}

void ROSThread::run() 
{
    ros::Rate rate(10);
    while(this->running) {
        ros::spinOnce();
        rate.sleep();
    }
}