#include <ros/ros.h>
#include <mission_planner/MainWindow.h>
#include <qapplication.h>

#include <qpushbutton.h>

int main( int argc, char **argv )
{
    QApplication a( argc, argv );
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle nh;
    MainWindow* window = new MainWindow(nh);
   
    window->show();
    return a.exec();
}