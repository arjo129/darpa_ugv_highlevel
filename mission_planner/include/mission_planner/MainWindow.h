#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <mission_planner/RosThread.h>
#include <QPixmap>
#include "ui_MainWindow.h"

class MainWindow : public QMainWindow {
    private:
        Ui::MainWindow* ui;
        QGraphicsScene* scene;
        ros::NodeHandle nh;
        ROSThread rosthread;
    public:
        MainWindow(ros::NodeHandle nh);
        ~MainWindow();
    public slots:
        void addPixmap(const QPixmap& map, int x, int y, float theta);
};
#endif