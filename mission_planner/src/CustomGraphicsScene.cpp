#include <mission_planner/CustomGraphicsScene.h>

void CustomGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    prevPixelPos = event->scenePos().y(); // reset pixel movement count for rotation on mouse click
    QGraphicsScene::mousePressEvent(event);
}

void CustomGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
        // Right Button movment controls rotation
        // Left Button movement controls translation
        if (event->buttons() == Qt::RightButton) {
            int currPixelPos = event->scenePos().y();

            // mouse moved downwards
            if (currPixelPos > prevPixelPos + ROTATION_PIXEL_THRESHOLD) {
                prevPixelPos = currPixelPos;
                emit onRotate(RotateState::CLOCKWISE);
            }
            // mouse moved upwards
            else if (currPixelPos < prevPixelPos - ROTATION_PIXEL_THRESHOLD) {
                prevPixelPos = currPixelPos;
                emit onRotate(RotateState::ANTI_CLOCKWISE);
            }
        }
        else if (event->buttons() == Qt::LeftButton) {
            QGraphicsScene::mouseMoveEvent(event);
        }

    }
