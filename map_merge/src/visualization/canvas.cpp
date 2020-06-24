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

void Helper::setScores(std::vector<double>& score) {
    this->_score.clear();
    for(int i = 0; i < score.size(); i++)
        this->_score.push_back(score[i]);
}

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

    double min_score = INFINITY, max_score = -INFINITY;
    for(auto score: _score) {
        min_score = std::min(score, min_score);
        max_score = std::max(score, max_score);
    }
    auto width = max_score - min_score;

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

    for(int i = 0; i < _centroids.size(); i++) {
        auto x_coord = scale * (_centroids[i].x - min_x);
        auto y_coord = scale * (_centroids[i].y - min_y);
        if(i >= _score.size()) continue;
        auto brush = QBrush(QColor::fromHsvF(-0.7*(_score[i] - min_score)/width + 1, 1, 1, 0.7));
        painter->setBrush(brush);
        painter->drawEllipse(QPointF(x_coord, y_coord), 10, 10);
    }

    auto brush = QBrush(QColor(255,255,255));
    painter->setBrush(brush);
    painter->drawEllipse(QPointF(scale*(this->robot_x - min_x), scale*(this->robot_y - min_y)), 10, 10);
}

void Helper::setLocation(double x, double y) {
    this->robot_x = x;
    this->robot_y = y;
}