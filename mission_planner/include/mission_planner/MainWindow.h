#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <mission_planner/RosThread.h>
#include <mission_planner/CustomGraphicsScene.h>
#include <QPixmap>
#include <QGraphicsItem>
#include <QTransform>
#include <QDebug>
#include <stack>
#include "ui_MainWindow.h"

enum class SliderState {
    EDITING,
    LIVE_VIEW
};

enum class EditorState {
    MOVE,
    ROTATE
};

// Struct to store data of the applied transform between laser scans
typedef struct {
    uint32_t startIdx;
    uint32_t endIdx;
    QPointF translationOffset;
    qreal rotationOffset;
    QTransform qTransform;
} offsetState;

class MainWindow : public QMainWindow {
    private:
        Ui::MainWindow* ui;
        CustomGraphicsScene* scene;
        ros::NodeHandle nh;
        ROSThread rosthread;
        std::vector<QGraphicsPixmapItem*> laserscans;
        std::stack<offsetState> offsetStack;
        SliderState sliderState;
        EditorState editorState;
        uint32_t currentIndex = 0;
        double prevYaw = 0;
        QPointF prevPos;
        QTransform getTransform(QPointF translation, double rotationAngle);
        void applyTransformList(int startIdx, int endIdx, 
                                QTransform transform, double rotationAngleTransform);
        void applyTransform(QGraphicsPixmapItem* item, 
                            QTransform transform, double rotationAngleTransform);
    public:
        MainWindow(ros::NodeHandle nh);
        ~MainWindow();
    public slots:
        void addPixmap(const QPixmap& map, int x, int y, float theta);
        void sliderMoved(int value);
        void propagateChanges();
        void rotatePixMap(RotateState rotateState);
};
#endif