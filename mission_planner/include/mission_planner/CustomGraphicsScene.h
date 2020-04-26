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
    QGraphicsPolygonItem* robot;
    CustomGraphicsScene() : QGraphicsScene() {
        QPolygonF polygon;
        polygon << QPointF(-5,0) << QPointF(0, 5) << QPointF(5, 0);
        robot = this->addPolygon(polygon);
    }
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
        ROS_INFO("Object at %.f, %.f", currentScan->scenePos().rx(), currentScan->scenePos().ry());
        auto position = currentScan->mapFromScene(event->scenePos()); 
        ROS_INFO("relative position %f %f", (250-position.rx())/25, (250-position.ry())/25);
        robot->rosthread.sendGoal((250-position.rx())/25, (250-position.ry())/25, 0);
    }
};
#endif /* CUSTOM_GRAPHICS_SCENCE_H */
