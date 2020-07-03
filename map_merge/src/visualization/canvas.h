#ifndef _CANVAS_H_
#define _CANVAS_H_
#include <QWidget>

#include <QBrush>
#include <QFont>
#include <QPen>
#include <QWidget>
#include <map_merge/centroids.h>
#include <vector>
//! [0]
class Helper
{
public:
    Helper(std::vector<Centroid> centroids);
    void paint(QPainter *painter, QPaintEvent *event, int elapsed);
    void setScores(std::vector<double>& score);
    void setLocation(double x, double y);

private:
    QBrush background;
    QBrush circleBrush;
    QFont textFont;
    QPen circlePen;
    QPen textPen;
    double robot_x, robot_y;
    std::vector<double> _score;
    std::vector<Centroid> _centroids;
};

class Canvas : public QWidget
{
    Q_OBJECT

public:
    Canvas(Helper *helper, QWidget *parent);

public slots:
    void animate();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    Helper *helper;
    int elapsed;
};


#endif