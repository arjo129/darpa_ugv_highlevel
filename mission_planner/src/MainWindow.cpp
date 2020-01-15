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
    connect(ui->progressSlider, &QSlider::sliderMoved, this, &MainWindow::sliderMoved);
    connect(ui->returnDraw, &QPushButton::pressed, this, &MainWindow::propagateChanges);
    connect(ui->rotateMode, &QPushButton::pressed, this, &MainWindow::enterRotateMode);
    connect(ui->rotateMode, &QPushButton::pressed, this, &MainWindow::enterMoveMode);
    this->sliderState = SliderState::LIVE_VIEW;
    this->editorState = EditorState::MOVE;
}

MainWindow::~MainWindow() {
    delete scene;
    delete ui;
}

void MainWindow::addPixmap(const QPixmap& map, int x, int y, float theta){
    QGraphicsPixmapItem* item = scene->addPixmap(map);
    laserscans.push_back(item);
    item->setPos(x,y);
    item->setTransformOriginPoint(map.rect().center());
    item->setRotation(theta*57.29);
    if(sliderState == SliderState::LIVE_VIEW){
        ui->progressSlider->setValue(laserscans.size());
        item->setVisible(true);
    }
    else{
        item->setVisible(false);
    }
    ui->progressSlider->setRange(0, laserscans.size());
}

void MainWindow::sliderMoved(int value) {
    sliderState = SliderState::EDITING;
    for (int i = 0; i < value; i++) {
        laserscans[i]->setVisible(true);
        laserscans[i]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsMovable, false);
        laserscans[i]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsSelectable, false);
    }
    for (int i = value; i < laserscans.size(); i++) {
        laserscans[i]->setVisible(false);
    }
    laserscans[value]->setVisible(true);
    laserscans[value]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsMovable, true);
    laserscans[value]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsSelectable, true);
    prevPos = laserscans[value]->pos();
    prevYaw = laserscans[value]->rotation();
    currentIndex = value;
}

void MainWindow::propagateChanges() {
    if(sliderState != SliderState::EDITING)
        return;
    
    QPointF centerPoint = laserscans[currentIndex]->pos();
    QPointF translation = centerPoint - prevPos;
    float rotation = laserscans[currentIndex]->rotation();
    float amountRotated = rotation - prevYaw;

    for(int i = currentIndex ; i < laserscans.size(); i++) {
        /*for(laserscans[i]){
            QPointF newPoint = translation + laserscans[i]->pos();
            QPointf displacementFromRootNode = newPoint - centerPoint;
            //displacement
        }*/
    }
}

void MainWindow::enterRotateMode() {
    std::cout << "rotate mode" << std::endl;
}

void MainWindow::enterMoveMode() {
    std::cout << "move mode" << std::endl;
}