#include "canvas.h"
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>

//! [0]
Canvas::Canvas(Helper *helper, QWidget *parent)
    : QWidget(parent), helper(helper)
{
    elapsed = 0;
}
//! [0]

//! [1]
void Canvas::animate()
{
    elapsed = (elapsed + qobject_cast<QTimer*>(sender())->interval()) % 1000;
    update();
}
//! [1]

//! [2]
void Canvas::paintEvent(QPaintEvent *event)
{
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    helper->paint(&painter, event, elapsed);
    painter.end();
}

//! [0]
Helper::Helper(std::vector<Centroid> centroids)
{
    QLinearGradient gradient(QPointF(50, -20), QPointF(80, 20));
    gradient.setColorAt(0.0, Qt::white);
    gradient.setColorAt(1.0, QColor(0xa6, 0xce, 0x39));

    background = QBrush(QColor(64, 32, 64));
    
    this->_centroids = centroids;
}
//! [0]

//! [1]
void Helper::paint(QPainter *painter, QPaintEvent *event, int elapsed)
{
    painter->fillRect(event->rect(), background);
    float max_x = -INFINITY, max_y = -INFINITY, min_x = INFINITY, min_y = INFINITY;

    //RESIZE FOR SCALE
    for(auto centroid: _centroids) {
        max_x = std::max(centroid.x, max_x);
        max_y = std::max(centroid.y, max_y);
        
        min_x = std::min(centroid.x, min_x);
        min_y = std::min(centroid.y, min_y);
    }


    auto map_width  = max_x - min_x;
    auto map_height = max_y - min_y;
    auto screen_width = event->rect().width();
    auto screen_height = event->rect().height();
    auto map_aspect_ratio = map_height/map_width;
    auto screen_aspect_ratio = screen_height/screen_width;

    double scale;
    if(screen_aspect_ratio > map_aspect_ratio) {
        scale = screen_width/map_width;
    }
    else {
        scale = screen_height/map_height;
    }

    for(auto centroid: _centroids) {
        auto x_coord = scale*(centroid.x - min_x);
        auto y_coord = scale*(centroid.y - min_y);
        auto brush = QBrush(QColor::fromHsvF(0.7, 1, 1));
        painter->setBrush(brush);
        painter->drawEllipse(QPointF(x_coord, y_coord), 10, 10);
    }
}