#include <mission_planner/MainWindow.h>

#define ROTATION_INCREMENT 1 // 1 degree increment per signal from onRotate()

MainWindow::MainWindow(ros::NodeHandle _nh): nh(_nh), rosthread(_nh) {
    rosthread.start();
    ui = new Ui::MainWindow();
    ui->setupUi(this);
    scene = new CustomGraphicsScene();
    ui->graphicsView->setScene(scene);
    ui->horizontalSplitPanel->setStretchFactor(0,2);
    qRegisterMetaType<QPixmap>("QPixmap");
    qRegisterMetaType<RotateState>("RotateState");
    connect(&rosthread, &ROSThread::scanRecieved, this, &MainWindow::addPixmap);
    connect(ui->progressSlider, &QSlider::sliderMoved, this, &MainWindow::sliderMoved);
    connect(ui->returnDraw, &QPushButton::pressed, this, &MainWindow::propagateChanges);
    connect(scene, &CustomGraphicsScene::onRotate, this, &MainWindow::rotatePixMap);
    this->sliderState = SliderState::LIVE_VIEW;
    this->editorState = EditorState::MOVE;
    offsetState initialState = {0, 0, QPointF(0.0, 0.0), 0.0};
    offsetStack.push(initialState); // initially no transform applied to map
    ROS_INFO("Starting UI");
}

MainWindow::~MainWindow() {
    delete scene;
    delete ui;
}

QTransform MainWindow::getTransform(QPointF translation, double rotationAngle) {
    QTransform qtf;
    qtf.translate(translation.x(), translation.y());
    qtf.rotate(offsetStack.top().rotationOffset);
    return qtf;
}

// Applies transform to laserscan list from start and end indexes (inclusive)
void MainWindow::applyTransformList(int startIdx, int endIdx, 
                            QTransform transform, double rotationAngleTransform) {

    if (startIdx < 0 || endIdx >= laserscans.size()) {
        ROS_WARN("applyTransform(): Out of bounds, not applying transform");
        return;
    } 

    for (int idx = startIdx;idx <= endIdx;idx++) {
        QGraphicsPixmapItem* item = laserscans[idx];
        applyTransform(item, transform, rotationAngleTransform);
    }
}

// Sets the new position and rotation of graphics item based on transform
void MainWindow::applyTransform(QGraphicsPixmapItem* item, 
                                QTransform transform, double rotationAngleTransform) {
    QPointF currentPos = item->pos();
    QPointF transformedPos = transform.map(currentPos);
    double currentRotation = item->rotation();
    double transformedRotation = currentRotation + rotationAngleTransform;
    item->setPos(transformedPos);
    item->setRotation(transformedRotation);
}

void MainWindow::addPixmap(const QPixmap& map, int x, int y, float theta) {
    QGraphicsPixmapItem* item = scene->addPixmap(map);
    item->setTransformOriginPoint(map.rect().center());

    // apply latest transform to all incoming maps
    QTransform transform = offsetStack.top().qTransform;
    double rotationAngleTransform = offsetStack.top().rotationOffset;
    applyTransform(item, transform, rotationAngleTransform);

    if (sliderState == SliderState::LIVE_VIEW) {
        ui->progressSlider->setValue(laserscans.size());
        item->setVisible(true);
    }
    else {
        item->setVisible(false);
    }
    ui->progressSlider->setRange(0, laserscans.size());
    laserscans.push_back(item);
}

void MainWindow::sliderMoved(int value) {
    ROS_INFO("Map Editing Mode");
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

    ROS_INFO("Propagating Map Changes");
    
    // update latest map transform from the UI and save on the stack
    QPointF centerPoint = laserscans[currentIndex]->pos();
    QPointF translation = (centerPoint - prevPos) + offsetStack.top().translationOffset;
    float rotation = std::fmod(laserscans[currentIndex]->rotation() - prevYaw + offsetStack.top().rotationOffset, 360);
    float amountRotated = rotation - prevYaw;
    QTransform transform = getTransform(translation, rotation);
    offsetState changes = {currentIndex, 0, translation, rotation, transform};
    offsetStack.top().endIdx = currentIndex - 1;
    offsetStack.push(changes);

    /** Apply transform to points that arrived while map was being edited,
     *  from index after the current edit point to the last received scan
     */ 
    applyTransformList(currentIndex+1, laserscans.size()-1, transform, rotation);

    sliderState = SliderState::LIVE_VIEW;
}

void MainWindow::rotatePixMap(RotateState rotateState) {
    if (sliderState == SliderState::EDITING) {
        qreal rotationAngle = laserscans[currentIndex]->rotation();
        if (rotateState == RotateState::CLOCKWISE) {
            rotationAngle += ROTATION_INCREMENT;
        }
        else if (rotateState == RotateState::ANTI_CLOCKWISE) {
            rotationAngle -= ROTATION_INCREMENT;
        }

        laserscans[currentIndex]->setRotation(rotationAngle);
    }
    
}
