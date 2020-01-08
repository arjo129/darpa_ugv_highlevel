#include <mission_planner/RosThread.h>
#include <QBitmap> 
#include <QImage>

ROSThread::ROSThread(ros::NodeHandle _nh): nh(_nh) {
    laserScanSub = _nh.subscribe("/scan", 10, &ROSThread::onLaserScan, this);
    this->running = true;
}

ROSThread::~ROSThread() {
    this->running = false;
}

void ROSThread::onLaserScan(sensor_msgs::LaserScan lscan) {
    static int x = 0;
    const int size = 500;
    float angle = lscan.angle_min;
    QImage* image = new QImage(size, size, QImage::Format::Format_RGB888);
    image->fill(QColor(Qt::white).rgb());
    for(int i = 0; i < lscan.ranges.size(); i++){
        if(lscan.ranges[i] > lscan.range_max) {
             angle+=lscan.angle_increment;
             continue;
        }
        int x = round(size/2*(lscan.ranges[i]/(lscan.range_max))*cos(angle))+size/2;
        int y = round(size/2*(lscan.ranges[i]/(lscan.range_max))*sin(angle))+size/2;
        image->setPixelColor(x,y,QColor(255,0,0));
        angle+=lscan.angle_increment;
    }
    x+=100;

    QPixmap pixmap = QBitmap::fromImage(*image);
    emit scanRecieved(pixmap,x,0,0);
}

void ROSThread::run() {
    ros::Rate rate(10);
    while(this->running) {
        ros::spinOnce();
        rate.sleep();
    }
}