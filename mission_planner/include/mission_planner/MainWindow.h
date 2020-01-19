#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <mission_planner/RosThread.h>
#include <mission_planner/CustomGraphicsScene.h>
#include <QPixmap>
#include <qgraphicsitem.h>
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
        CustomGraphicsScene* scene;
        ros::NodeHandle nh;
        ROSThread rosthread;
        std::vector<QGraphicsPixmapItem*> laserscans;
        SliderState sliderState;
        EditorState editorState;
        int currentIndex = 0;
        double prevYaw = 0;
        QPointF prevPos;
    public:
        MainWindow(ros::NodeHandle nh);
        ~MainWindow();
    public slots:
        void addPixmap(const QPixmap& map, int x, int y, float theta);
        void sliderMoved(int value);
        void propagateChanges();
        void enterRotateMode();
        void enterMoveMode();
        void rotatePixMap(RotateState rotateState);
};
#endif