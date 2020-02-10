#ifndef CUSTOM_GRAPHICS_SCENCE_H
#define CUSTOM_GRAPHICS_SCENCE_H

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

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

#endif /* CUSTOM_GRAPHICS_SCENCE_H */
