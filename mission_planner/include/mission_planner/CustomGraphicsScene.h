#ifndef CUSTOM_GRAPHICS_SCENCE_H
#define CUSTOM_GRAPHICS_SCENCE_H

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <mission_planner/Robot.h>
#include <ros/ros.h>

#define ROTATION_PIXEL_THRESHOLD 3 // how many vertical pixels after which it is considered movement

enum class RotateState {
    CLOCKWISE,
    ANTI_CLOCKWISE
};

class CustomGraphicsScene : public QGraphicsScene {
    Q_OBJECT
public:
    int prevPixelPos = -1;
    CustomGraphicsScene() : QGraphicsScene() { }
public slots:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
signals:
    void onRotate(RotateState rotateState);
};

class LaserScanView : public QGraphicsScene {
    Q_OBJECT
public:
    int prevPixelPos = -1;
    Robot* robot;
    QGraphicsPixmapItem * currentScan;
    LaserScanView(Robot* robot) : QGraphicsScene() { 
        robot = robot;
        currentScan = this->addPixmap(QPixmap());
    }
public slots:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) {
        ROS_INFO("Mouse Pressed at %f,%f", event->scenePos().rx(), event->scenePos().ry());
        //robot->rosthread.sendRobotGoal();
    }
};
#endif /* CUSTOM_GRAPHICS_SCENCE_H */
