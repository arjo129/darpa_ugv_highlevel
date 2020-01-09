#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <mission_planner/RosThread.h>
#include <QPixmap>
#include "ui_MainWindow.h"
enum class SliderState {
    EDITING,
    LIVE_VIEW
};

enum class EditorState {
    MOVE,
    ROTATE
};
class MainWindow : public QMainWindow {
    private:
        Ui::MainWindow* ui;
        QGraphicsScene* scene;
        ros::NodeHandle nh;
        ROSThread rosthread;
        std::vector<QGraphicsPixmapItem*> laserscans;
        SliderState sliderState;
        EditorState editorState;
    public:
        MainWindow(ros::NodeHandle nh);
        ~MainWindow();
    public slots:
        void addPixmap(const QPixmap& map, int x, int y, float theta);
        void sliderMoved(int value);
};
#endif