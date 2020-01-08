#include <mission_planner/MainWindow.h>
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>

MainWindow::MainWindow(ros::NodeHandle _nh): nh(_nh), rosthread(_nh) {
    rosthread.start();
    ui = new Ui::MainWindow();
    ui->setupUi(this);
    scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);
    qRegisterMetaType<QPixmap>("QPixmap");
    connect(&rosthread, &ROSThread::scanRecieved, this, &MainWindow::addPixmap);
}

MainWindow::~MainWindow() {
    delete scene;
    delete ui;
}

void MainWindow::addPixmap(const QPixmap& map, int x, int y, float theta){
    QGraphicsPixmapItem* item = scene->addPixmap(map);
    item->setPos(x,y);
    item->setTransformOriginPoint(map.rect().center());
    item->setRotation(theta*57.29);
}