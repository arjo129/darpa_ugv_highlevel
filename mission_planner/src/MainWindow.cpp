#include <mission_planner/MainWindow.h>

MainWindow::MainWindow(ros::NodeHandle _nh): nh(_nh), darpaServerThread(_nh) {
    qRegisterMetaType<QPixmap>("QPixmap");
    qRegisterMetaType<RotateState>("RotateState");
    qRegisterMetaType<std::string>("std::string");

    darpaServerThread.start();

    ui = new Ui::MainWindow();
    ui->setupUi(this);
    MainWindow::showMaximized();

    initRobots();
    // initMapUI();
    initDarpaInterfaceUI();
    initEStopUI();
    initGoalUI();

    ROS_INFO("Starting UI");

    // // create a scene and add it your view
    scene = new CustomGraphicsScene;
    ui->map->setScene(scene);


    // Add the vertical lines first, paint them red
    for (int x=0; x<=1000; x+=50)
        scene->addLine(x,0,x,1000, QPen(Qt::red));

    // Now add the horizontal lines, paint them green
    for (int y=0; y<=1000; y+=50)
        scene->addLine(0,y,1000,y, QPen(Qt::green));

    connect(scene, &CustomGraphicsScene::clickedPos, this, &MainWindow::clickedPos);
    // // Fit the view in the scene's bounding rect
    // ui->map->fitInView(scene->itemsBoundingRect());

}

void MainWindow::clickedPos(int x, int y) {
    std::cout << x << y <<std::endl;
}

MainWindow::~MainWindow() {
    // delete ui;

    // for(int i=0;i<NUM_ROBOTS;i++) {
    //     delete scenes[i];
    //     delete robots[i];
    // }

    // delete[] scenes;
    // delete[] robots; 
}

void MainWindow::switchLocalGlobal(){
    // this->showGlobalMap = !this->showGlobalMap;
    ROS_INFO("toggling between local and global map");
    // if(this->showGlobalMap) {
    //     ui->graphicsView_1->setScene(scenes[0]);
    //     ui->graphicsView_2->setScene(scenes[1]);
    //     ui->graphicsView_3->setScene(scenes[2]);
    //     ui->graphicsView_4->setScene(scenes[3]);
    //     ui->graphicsView_5->setScene(scenes[4]);
    // }
    // else {
    //     ui->graphicsView_1->setScene(currentScanScene[0]);
    //     ui->graphicsView_2->setScene(currentScanScene[1]);
    //     ui->graphicsView_3->setScene(currentScanScene[2]);
    //     ui->graphicsView_4->setScene(currentScanScene[3]);
    //     ui->graphicsView_5->setScene(currentScanScene[4]);
    // }
}
void MainWindow::initMapUI() {

    // scenes = new CustomGraphicsScene*[NUM_ROBOTS];
    // currentScanScene = new LaserScanView*[NUM_ROBOTS];
    // for (int idx = 0; idx < NUM_ROBOTS; idx++) {
    //     scenes[idx] = new CustomGraphicsScene();
    //     currentScanScene[idx] = new LaserScanView(robots[idx]);
    // }

    // ui->graphicsView_1->setScene(scenes[0]);
    // ui->graphicsView_2->setScene(scenes[1]);
    // ui->graphicsView_3->setScene(scenes[2]);
    // ui->graphicsView_4->setScene(scenes[3]);
    // ui->graphicsView_5->setScene(scenes[4]);

    // // TODO: Connect slider and map rotation to individual maps
    // connect(ui->progressSlider, &QSlider::sliderMoved, this, &MainWindow::sliderMoved);
    // connect(ui->returnDraw, &QPushButton::pressed, this, &MainWindow::propagateChanges);
    // connect(scenes[0], &CustomGraphicsScene::onRotate, this, &MainWindow::rotatePixMap);
    // connect(ui->actionLocal_Map, &QAction::triggered, this, &MainWindow::switchLocalGlobal);

    // this->sliderState = SliderState::LIVE_VIEW;
    // this->editorState = EditorState::MOVE;
    // offsetState initialState = {0, 0, QPointF(0.0, 0.0), 0.0};
    // offsetStack.push(initialState); // initially no transform applied to map
}

// initialize the rosthread for each robot
void MainWindow::initRobots() {
    selectRobot1BtnClicked();
    robots = new Robot*[NUM_ROBOTS];
    for (int robotId = 1; robotId <= NUM_ROBOTS; robotId++) {
        robots[robotId-1] = new Robot(nh, robotId);
    //     connect(&(robots[idx-1]->rosthread), &ROSThread::scanRecieved, this, &MainWindow::addPixmap);
        connect(&(robots[robotId-1]->rosthread), &ROSThread::artifactReceived, this, &MainWindow::artifactReceived);
        connect(&(robots[robotId-1]->rosthread), &ROSThread::rosOutReceived, this, &MainWindow::rosOutReceived);
    }
    connect(ui->selectRobot1BtnClicked, &QPushButton::clicked, this, &MainWindow::selectRobot1BtnClicked);
    connect(ui->selectRobot2BtnClicked, &QPushButton::clicked, this, &MainWindow::selectRobot2BtnClicked);
    connect(ui->selectRobot3BtnClicked, &QPushButton::clicked, this, &MainWindow::selectRobot3BtnClicked);
    connect(ui->selectRobot4BtnClicked, &QPushButton::clicked, this, &MainWindow::selectRobot4BtnClicked);
    connect(ui->selectRobot5BtnClicked, &QPushButton::clicked, this, &MainWindow::selectRobot5BtnClicked);
}

void MainWindow::initDarpaInterfaceUI() {
    connect(&darpaServerThread, &DarpaServerThread::darpaStatusRecieved, this, &MainWindow::darpaStatusRecieved);
    connect(&darpaServerThread, &DarpaServerThread::artifactStatusReceived, this, &MainWindow::artifactStatusReceived);
    connect(&darpaServerThread, &DarpaServerThread::mapUpdateReceived, this, &MainWindow::mapUpdateReceived);
    connect(this, &MainWindow::reportArtifact, &darpaServerThread, &DarpaServerThread::reportArtifact);
    connect(ui->reportArtifactBtnClicked, &QPushButton::pressed, this, &MainWindow::reportArtifactBtnClicked);
}

void MainWindow::initEStopUI() {
    connect(ui->startBtnClicked, &QPushButton::clicked, this, &MainWindow::startBtnClicked);
    connect(ui->stopBtnClicked, &QPushButton::clicked, this, &MainWindow::stopBtnClicked);
    connect(ui->startAllBtnClicked, &QPushButton::clicked, this, &MainWindow::startAllBtnClicked);
    connect(ui->stopAllBtnClicked, &QPushButton::clicked, this, &MainWindow::stopAllBtnClicked);
}

void MainWindow::initGoalUI() {
    goalXInput = ui->goalXInput;
    goalYInput = ui->goalYInput;
    goalThetaInput = ui->goalThetaInput;

    artifactXInput = ui->artifactXInput;
    artifactYInput = ui->artifactYInput;
    artifactTypeInput = ui->artifactTypeInput;
    ROS_INFO("Initializing LORA and GOAL Functionality");

    QStringList wordList;
    wordList << "Survivor" << "Cell Phone" << "Backpack" << "Drill" << "Fire Extinguisher" << "Gas" << "Vent";
    completer = new QCompleter(wordList, this);
    completer->setCaseSensitivity(Qt::CaseInsensitive);
    ui->artifactTypeInput->setCompleter(completer);

    connect(ui->sendGoalBtnClicked, &QPushButton::clicked, this, &MainWindow::sendGoalBtnClicked);
    connect(ui->loraDropBtnClicked, &QPushButton::clicked, this, &MainWindow::loraDropBtnClicked);
}

QVector3D MainWindow::getArtifactPos() {
    QString xInput = artifactXInput->text();
    QString yInput = artifactYInput->text();

    double x = xInput.toDouble();
    double y = yInput.toDouble();

    return QVector3D(x,y,0);
}

QVector3D MainWindow::getRobotGoalPos() {
    QString xInput =  goalXInput->text();
    QString yInput = goalYInput->text();
    QString thetaInput = goalThetaInput->text();

    double x = xInput.toDouble();
    double y = yInput.toDouble();
    double z = thetaInput.toDouble(); // Note that z refers to yaw and not height

    return QVector3D(x, y, z);
}

std::string MainWindow::getArtifactType() {
    std::string typeInput = artifactTypeInput->text().toStdString();
    return typeInput;
}

QTransform MainWindow::getTransform(QPointF translation, double rotationAngle) {
    QTransform qtf;
    // qtf.translate(translation.x(), translation.y());
    // qtf.rotate(offsetStack.top().rotationOffset);
    return qtf;
}

// Applies transform to laserscan list from start and end indexes (inclusive)
void MainWindow::applyTransformList(std::vector<QGraphicsPixmapItem*> laserscans, int startIdx, int endIdx, QTransform transform, double rotationAngleTransform) {

    // if (startIdx < 0 || endIdx >= laserscans.size()) {
    //     ROS_WARN("applyTransform(): Out of bounds, not applying transform");
    //     return;
    // } 

    // for (int idx = startIdx;idx <= endIdx;idx++) {
    //     QGraphicsPixmapItem* item = laserscans[idx];
    //     applyTransform(item, transform, rotationAngleTransform);
    // }
}

// Sets the new position and rotation of graphics item based on transform
void MainWindow::applyTransform(QGraphicsPixmapItem* item, 
                                QTransform transform, double rotationAngleTransform) {
    // QPointF currentPos = item->pos();
    // QPointF transformedPos = transform.map(currentPos);
    // double currentRotation = item->rotation();
    // double transformedRotation = currentRotation + rotationAngleTransform;
    // item->setPos(transformedPos);
    // item->setRotation(transformedRotation);
}

// TODO: Slider and return button still configured for map 1, 
//       find a way to extend UI editing for all N maps
void MainWindow::addPixmap(int robotNum, const QPixmap& map, float x, float y, float theta) {
    // if (std::isnan(theta)) {
    //     ROS_ERROR("Yaw of map provided is NAN. Rejecting.");
    //     return;
    // }

    // ROS_INFO("Laser Scan Received: robot_%d", robotNum);
    // CustomGraphicsScene* scene = scenes[robotNum - 1];
    // QGraphicsPixmapItem* item = scene->addPixmap(map);
    // currentScanScene[robotNum - 1]->currentScan->setPixmap(map);
    // scenes[robotNum - 1]->robot->setPos(x,y);
    // scenes[robotNum - 1]->robot->setRotation(theta*57.92);
    // item->setPos(x,y);
    // item->setRotation(theta*57.29);
    // item->setTransformOriginPoint(map.rect().center());
    // scenes[robotNum - 1]->robot->setTransformOriginPoint(map.rect().center());

    

    // // apply latest transform to all incoming maps
    // QTransform transform = offsetStack.top().qTransform;
    // double rotationAngleTransform = offsetStack.top().rotationOffset;
    // applyTransform(item, transform, rotationAngleTransform);

    // if (sliderState == SliderState::LIVE_VIEW) {
    //     ui->progressSlider->setValue(robots[robotNum-1]->laserscans.size());
    //     item->setVisible(true);
    // }
    // else {
    //     item->setVisible(false);
    // } 
    // ui->progressSlider->setRange(0, robots[robotNum-1]->laserscans.size());
    // robots[robotNum-1]->laserscans.push_back(item);
}


void MainWindow::sliderMoved(int value) {
    // ROS_INFO("Map Editing Mode");
    // sliderState = SliderState::EDITING;

    // int robotNum = 1; // temp hack, PLS FIX LATER
    // std::vector<QGraphicsPixmapItem*> laserscans = robots[robotNum-1]->laserscans;

    // for (int i = 0; i < value; i++) {
    //     laserscans[i]->setVisible(true);
    //     laserscans[i]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsMovable, false);
    //     laserscans[i]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsSelectable, false);
    // }
    // for (int i = value; i < laserscans.size(); i++) {
    //     laserscans[i]->setVisible(false);
    // }
    // laserscans[value]->setVisible(true);
    // laserscans[value]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsMovable, true);
    // laserscans[value]->setFlag(QGraphicsItem::GraphicsItemFlag::ItemIsSelectable, true);
    // prevPos = laserscans[value]->pos();
    // prevYaw = laserscans[value]->rotation();
    // currentIndex = value;
}

void MainWindow::propagateChanges() {
    // if(sliderState != SliderState::EDITING)
    //     return;

    // ROS_INFO("Propagating Map Changes");

    // int robotNum = 1; // temp hack, PLS FIX LATER
    // std::vector<QGraphicsPixmapItem*> laserscans = robots[robotNum-1]->laserscans;
    
    // update latest map transform from the UI and save on the stack
    // QPointF centerPoint = laserscans[currentIndex]->pos();
    // QPointF translation = (centerPoint - prevPos) + offsetStack.top().translationOffset;
    // float rotation = std::fmod(laserscans[currentIndex]->rotation() - prevYaw + offsetStack.top().rotationOffset, 360);
    // float amountRotated = rotation - prevYaw;
    // QTransform transform = getTransform(translation, rotation);
    // offsetState changes = {currentIndex, 0, translation, rotation, transform};
    // offsetStack.top().endIdx = currentIndex - 1;
    // offsetStack.push(changes);

    /** Apply transform to points that arrived while map was being edited,
     *  from index after the current edit point to the last received scan
     */ 
    // applyTransformList(laserscans, currentIndex+1, laserscans.size()-1, transform, rotation);

    // sliderState = SliderState::LIVE_VIEW;
}

void MainWindow::rotatePixMap(RotateState rotateState) {

    // int robotNum = 1; // temp hack, PLS FIX LATER
    // std::vector<QGraphicsPixmapItem*> laserscans = robots[robotNum-1]->laserscans;

    // if (sliderState == SliderState::EDITING) {
    //     qreal rotationAngle = laserscans[currentIndex]->rotation();
    //     if (rotateState == RotateState::CLOCKWISE) {
    //         rotationAngle += ROTATION_INCREMENT;
    //     }
    //     else if (rotateState == RotateState::ANTI_CLOCKWISE) {
    //         rotationAngle -= ROTATION_INCREMENT;
    //     }

    //     laserscans[currentIndex]->setRotation(rotationAngle);
    // }
    
}

void MainWindow::mapUpdateReceived(bool success, std::string errorStr) {

}

void MainWindow::artifactReceived(float x, float y, float z, std::string details) {
    std:: string artifactLogsMessage = "Received " + details + " at (" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")";
    QString qstr = QString::fromStdString(artifactLogsMessage);
    ui->artifactLogs->append(qstr);
}


void MainWindow::reportArtifactBtnClicked() {
    ROS_INFO("Reporting artifact on Robot %d.", activeRobotId);
    QVector3D pos3d = getArtifactPos();
    std::string artifactType = getArtifactType();
    emit reportArtifact(pos3d.x(), pos3d.y(), pos3d.z(), artifactType);
    ROS_INFO("Reported artifact on Robot %d", activeRobotId);
}

void MainWindow::sendGoalBtnClicked() {
    ROS_INFO("Sending goal to Robot %d.", activeRobotId);
    if (activeRobotId < 1 || activeRobotId > NUM_ROBOTS) {
        ROS_ERROR("Robot %d does not exist.", activeRobotId);
        return;
    }
    QVector3D pos = getRobotGoalPos();
    robots[activeRobotId-1]->rosthread.sendGoal(pos.x(), pos.y(), pos.z());
    ROS_INFO("Sent Robot %d to X ,Y ,Theta: (%f, %f, %f)", activeRobotId, pos.x(), pos.y(), pos.z());
}

void MainWindow::loraDropBtnClicked() {
    ROS_INFO("Droppring LORA on Robot %d.", activeRobotId);
    if (activeRobotId < 1 || activeRobotId > NUM_ROBOTS) {
        ROS_ERROR("Robot %d does not exist.", activeRobotId);
        return;
    }
    robots[activeRobotId-1]->rosthread.dropLora();
    ROS_INFO("Dropped LORA on Robot %d", activeRobotId);
}

void MainWindow::darpaStatusRecieved(std::string teamName, double currentTime, int32_t numReportsLeft, int32_t currentScore) {
    ui->teamNameText->setText(QString::fromStdString(teamName));
    ui->currentTimeText->setText(QString::fromStdString(std::to_string(currentTime)));
    ui->reportsLeftText->setText(QString::fromStdString(std::to_string(numReportsLeft)));
    ui->scoreText->setText(QString::fromStdString(std::to_string(currentScore)));
}

void MainWindow::rosOutReceived(std::string msg, std::string name, std::string func, int robotNum) {
    if (robotNum == activeRobotId) {
        std::string logMessage = "Robot" + std::to_string(robotNum) + ": " + func + " " + name + " " + msg;
        QString qstr = QString::fromStdString(logMessage);
        ui->logs->append(qstr);
    }
}

void MainWindow::artifactStatusReceived(std::string result) {
    QString qstr = QString::fromStdString(result);
    ui->artifactLogs->append(qstr);
}

void MainWindow::stopBtnClicked() {
    robots[activeRobotId-1]->rosthread.stop();
}

void MainWindow::startBtnClicked() {
    robots[activeRobotId-1]->rosthread.start();
}

void MainWindow::stopAllBtnClicked() {
    for (int robotId = 0; robotId < NUM_ROBOTS; robotId++) {
        robots[robotId]->rosthread.stop();
    }
}

void MainWindow::startAllBtnClicked() {
    for (int robotId = 0; robotId < NUM_ROBOTS; robotId++) {
        robots[robotId]->rosthread.start();
    }
}

void MainWindow::selectRobot1BtnClicked() {
    handleSelectRobotBtnClicked(1);
}

void MainWindow::selectRobot2BtnClicked() {
    handleSelectRobotBtnClicked(2);
}

void MainWindow::selectRobot3BtnClicked() {
    handleSelectRobotBtnClicked(3);
}

void MainWindow::selectRobot4BtnClicked() {
    handleSelectRobotBtnClicked(4);
}

void MainWindow::selectRobot5BtnClicked() {
    handleSelectRobotBtnClicked(5);
}

void MainWindow::handleSelectRobotBtnClicked(int robotNum) {
    activeRobotId = robotNum;

    if (robotNum == 1) {
        ui->selectRobot1BtnClicked->setFont(QFont("Ubuntu", 13, QFont::Bold));
    } else {
        ui->selectRobot1BtnClicked->setFont(QFont("Ubuntu", 11));
    }

    if (robotNum == 2) {
        ui->selectRobot2BtnClicked->setFont(QFont("Ubuntu", 13, QFont::Bold));
    } else {
        ui->selectRobot2BtnClicked->setFont(QFont("Ubuntu", 11));
    }

    if (robotNum == 3) {
        ui->selectRobot3BtnClicked->setFont(QFont("Ubuntu", 13, QFont::Bold));
    } else {
        ui->selectRobot3BtnClicked->setFont(QFont("Ubuntu", 11));
    }
    
    if (robotNum == 4) {
        ui->selectRobot4BtnClicked->setFont(QFont("Ubuntu", 13, QFont::Bold));
    } else {
        ui->selectRobot4BtnClicked->setFont(QFont("Ubuntu", 11));
    }

    if (robotNum == 5) {
        ui->selectRobot5BtnClicked->setFont(QFont("Ubuntu", 13, QFont::Bold));
    } else {
        ui->selectRobot5BtnClicked->setFont(QFont("Ubuntu", 11));
    }
}