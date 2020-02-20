#include <mission_planner/MainWindow.h>

MainWindow::MainWindow(ros::NodeHandle _nh): nh(_nh), darpaServerThread(_nh) {
    qRegisterMetaType<QPixmap>("QPixmap");
    qRegisterMetaType<RotateState>("RotateState");
    qRegisterMetaType<std::string>("std::string");

    initRobots();
    darpaServerThread.start();
    ui = new Ui::MainWindow();
    ui->setupUi(this);
    
    initMapUI();
    initDarpaInterfaceUI();
    initEStopUI();
    initGoalUI();

    ui->horizontalSplitPanel->setStretchFactor(0,2);

    ROS_INFO("Starting UI");
}

MainWindow::~MainWindow() {
    delete ui;

    for(int i=0;i<NUM_ROBOTS;i++) {
        delete scenes[i];
        delete robots[i];
        delete eStopButtons[2*i];
        delete eStopButtons[2*i + 1];
    }

    delete[] scenes;
    delete[] robots; 
    delete[] eStopButtons;
}

void MainWindow::initMapUI() {

    scenes = new CustomGraphicsScene*[NUM_ROBOTS];
    for (int idx = 0; idx < NUM_ROBOTS; idx++) {
        scenes[idx] = new CustomGraphicsScene();
    }

    ui->graphicsView_1->setScene(scenes[0]);
    ui->graphicsView_2->setScene(scenes[1]);
    ui->graphicsView_3->setScene(scenes[2]);
    ui->graphicsView_4->setScene(scenes[3]);
    ui->graphicsView_5->setScene(scenes[4]);

    // TODO: Connect slider and map rotation to individual maps
    connect(ui->progressSlider, &QSlider::sliderMoved, this, &MainWindow::sliderMoved);
    connect(ui->returnDraw, &QPushButton::pressed, this, &MainWindow::propagateChanges);
    connect(scenes[0], &CustomGraphicsScene::onRotate, this, &MainWindow::rotatePixMap);

    this->sliderState = SliderState::LIVE_VIEW;
    this->editorState = EditorState::MOVE;
    offsetState initialState = {0, 0, QPointF(0.0, 0.0), 0.0};
    offsetStack.push(initialState); // initially no transform applied to map
}

// initialize the rosthread for each robot
void MainWindow::initRobots() {
    robots = new Robot*[NUM_ROBOTS];
    for (int idx = 1; idx <= NUM_ROBOTS; idx++) {
        robots[idx-1] = new Robot(nh, idx);
        connect(&(robots[idx-1]->rosthread), &ROSThread::scanRecieved, this, &MainWindow::addPixmap);
        connect(&(robots[idx-1]->rosthread), &ROSThread::artifactReceived, this, &MainWindow::addArtifactData);
    }
}

void MainWindow::initDarpaInterfaceUI() {
    artifactXBox = ui->artifactXInput;
    artifactYBox = ui->artifactYInput;
    artifactZBox = ui->artifactZInput;
    comboBoxArtifactType = ui->comboBoxArtifactType;

    connect(&darpaServerThread, &DarpaServerThread::darpaStatusRecieved, this, &MainWindow::darpaStatusRecieved);
    connect(&darpaServerThread, &DarpaServerThread::artifactStatusReceived, this, &MainWindow::artifactStatusReceived);
    connect(&darpaServerThread, &DarpaServerThread::mapUpdateReceived, this, &MainWindow::mapUpdateReceived);
    connect(this, &MainWindow::reportArtifact, &darpaServerThread, &DarpaServerThread::reportArtifact);
    connect(ui->reportArtifactBtn, &QPushButton::pressed, this, &MainWindow::artifactBtnClicked);
}

void MainWindow::initEStopUI() {
    ui->robot1StartBtn->setType(false, 1);
    ui->robot2StartBtn->setType(false, 2);
    ui->robot3StartBtn->setType(false, 3);
    ui->robot4StartBtn->setType(false, 4);
    ui->robot5StartBtn->setType(false, 5);

    connect(ui->robot1StartBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot2StartBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot3StartBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot4StartBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot5StartBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);

    ui->robot1StopBtn->setType(true, 1);
    ui->robot2StopBtn->setType(true, 2);
    ui->robot3StopBtn->setType(true, 3);
    ui->robot4StopBtn->setType(true, 4);
    ui->robot5StopBtn->setType(true, 5);

    connect(ui->robot1StopBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot2StopBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot3StopBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot4StopBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);
    connect(ui->robot5StopBtn, &EStopButton::clicked, this, &MainWindow::eStopBtnClicked);

    connect(ui->eStopAllBtn, &QPushButton::clicked, this, &MainWindow::eStopAllBtnClicked);
    connect(ui->startAllBtn, &QPushButton::clicked, this, &MainWindow::startAllBtnClicked);
}

void MainWindow::initGoalUI() {
    goalXBox = ui->goalXInput;
    goalYBox = ui->goalYInput;
    goalThetaBox = ui->goalThetaInput;
    comboBoxGoalRobotNum = ui->comboBoxGoalRobotNum;

    connect(ui->sendGoalBtn, &QPushButton::pressed, this, &MainWindow::sendGoalBtnClicked);
    connect(ui->loraDropBtn, &QPushButton::pressed, this, &MainWindow::loraDropBtnClicked);
}

QVector3D MainWindow::getArtifactPos() {

    double x = artifactXBox->value();
    double y = artifactYBox->value();
    double z = artifactZBox->value();

    return QVector3D(x,y,z);
}

QVector3D MainWindow::getRobotGoalPos() {

    double x = goalXBox->value();
    double y = goalYBox->value();
    double z = goalThetaBox->value();

    return QVector3D(x,y,z);
}

std::string MainWindow::getArtifactTypeStr() {
    std::string artifactTypeStr = (comboBoxArtifactType->currentText()).toStdString();
    ROS_INFO("%s artifact type selected", artifactTypeStr.c_str());
    return artifactTypeStr;
}

QTransform MainWindow::getTransform(QPointF translation, double rotationAngle) {
    QTransform qtf;
    qtf.translate(translation.x(), translation.y());
    qtf.rotate(offsetStack.top().rotationOffset);
    return qtf;
}

// Applies transform to laserscan list from start and end indexes (inclusive)
void MainWindow::applyTransformList(std::vector<QGraphicsPixmapItem*> laserscans, 
                                    int startIdx, int endIdx, QTransform transform, 
                                                    double rotationAngleTransform) {

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

// TODO: Slider and return button still configured for map 1, 
//       find a way to extend UI editing for all N maps
void MainWindow::addPixmap(int robotNum, const QPixmap& map, float x, float y, float theta) {
    if (std::isnan(theta)) {
        ROS_ERROR("Yaw of map provided is NAN. Rejecting.");
        return;
    }

    ROS_INFO("Laser Scan Received: robot_%d", robotNum);
    CustomGraphicsScene* scene = scenes[robotNum - 1];
    QGraphicsPixmapItem* item = scene->addPixmap(map);
    item->setPos(x,y);
    item->setRotation(theta*57.29);
    item->setTransformOriginPoint(map.rect().center());

    // apply latest transform to all incoming maps
    QTransform transform = offsetStack.top().qTransform;
    double rotationAngleTransform = offsetStack.top().rotationOffset;
    applyTransform(item, transform, rotationAngleTransform);

    if (sliderState == SliderState::LIVE_VIEW) {
        ui->progressSlider->setValue(robots[robotNum-1]->laserscans.size());
        item->setVisible(true);
    }
    else {
        item->setVisible(false);
    }
    ui->progressSlider->setRange(0, robots[robotNum-1]->laserscans.size());
    robots[robotNum-1]->laserscans.push_back(item);
}

void MainWindow::addArtifactData(float x, float y, float z, std::string details) {
    // QLabel *label = new QLabel;
    QString displayText = "Pos: " + QString::number(x, 'f', 2) + ", " + QString::number(y, 'f', 2) + \
                                    ", " +QString::number(z, 'f', 2) + ", " + QString::fromUtf8(details.c_str()) + "\n";
    // label->setAlignment(Qt::AlignTop);
    // label->setText(displayText);
    // ui->artifactVLayout->setWidget(label);
    // ui->artifactScrollArea->setWidgetResizable(false);
    // ui->artifactScrollArea->setWidget(ui->artifactLabel);
    // ui->artifactLabel->setWordWrap(true);
    // ui->artifactLabel->setText(ui->artifactLabel->text() + displayText);
    ui->artifactTextEdit->insertPlainText(displayText);
}

void MainWindow::sliderMoved(int value) {
    ROS_INFO("Map Editing Mode");
    sliderState = SliderState::EDITING;

    int robotNum = 1; // temp hack, PLS FIX LATER
    std::vector<QGraphicsPixmapItem*> laserscans = robots[robotNum-1]->laserscans;

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

    int robotNum = 1; // temp hack, PLS FIX LATER
    std::vector<QGraphicsPixmapItem*> laserscans = robots[robotNum-1]->laserscans;
    
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
    applyTransformList(laserscans, currentIndex+1, laserscans.size()-1, transform, rotation);

    sliderState = SliderState::LIVE_VIEW;
}

void MainWindow::rotatePixMap(RotateState rotateState) {

    int robotNum = 1; // temp hack, PLS FIX LATER
    std::vector<QGraphicsPixmapItem*> laserscans = robots[robotNum-1]->laserscans;

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

void MainWindow::artifactBtnClicked() {
    QVector3D pos3d = getArtifactPos();
    std::string artifactTypeStr = getArtifactTypeStr();
    emit reportArtifact(pos3d.x(), pos3d.y(), pos3d.z(), artifactTypeStr);
}

void MainWindow::sendGoalBtnClicked() {
    std::string robotName = (comboBoxGoalRobotNum->currentText()).toStdString();
    int robotNum = robotName.back() - '0';

    if (robotNum < 1 || robotNum > NUM_ROBOTS) {
        ROS_ERROR("Goal given to non-existent robot_%d\nAborting goal.", robotNum);
        return;
    }

    QVector3D pos = getRobotGoalPos(); // z refers to theta, not height
    robots[robotNum-1]->rosthread.sendRobotGoal(pos.x(), pos.y(), pos.z());
    ROS_INFO("Sent Goal X,Y,Theta: (%f, %f, %f) to %s", pos.x(), pos.y(), pos.z(), robotName.c_str());
}

void MainWindow::loraDropBtnClicked() {
    std::string robotName = (comboBoxGoalRobotNum->currentText()).toStdString();
    int robotNum = robotName.back() - '0';

    if (robotNum < 1 || robotNum > NUM_ROBOTS) {
        ROS_ERROR("LORA Drop command given to non-existent robot_%d\nAborting drop.", robotNum);
        return;
    }

    ROS_INFO("UI Triggering robot_%d lora drop", robotNum);
    robots[robotNum-1]->rosthread.dropLoraNode();
}

void MainWindow::darpaStatusRecieved(std::string teamName, double currentTime, 
                                 int32_t numReportsLeft, int32_t currentScore) {

}
void MainWindow::artifactStatusReceived(std::string result) {
    QMessageBox* resultDialog = new QMessageBox(this);
    resultDialog->setText(QString::fromStdString(result));
    resultDialog->show();
    resultDialog->raise();
    resultDialog->activateWindow();
}

void MainWindow::mapUpdateReceived(bool success, std::string errorStr) {

}

void MainWindow::eStopBtnClicked(bool isEStop, int robotNum) {
    if (isEStop) {
        robots[robotNum-1]->rosthread.eStopRobot();
    }
    else {
        robots[robotNum-1]->rosthread.startRobot();
    }
}

void MainWindow::eStopAllBtnClicked() {
    for (int idx = 0;idx < NUM_ROBOTS; idx++) {
        robots[idx]->rosthread.eStopRobot();
    }
}

void MainWindow::startAllBtnClicked() {
    for (int idx = 0;idx < NUM_ROBOTS; idx++) {
        robots[idx]->rosthread.startRobot();
    }
}

