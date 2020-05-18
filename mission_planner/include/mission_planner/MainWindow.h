#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <mission_planner/RosThread.h>
#include <mission_planner/DarpaServerThread.h>
#include <mission_planner/CustomGraphicsScene.h>
#include <mission_planner/EStopButton.h>
#include <mission_planner/Robot.h>
#include <mission_planner/Config.h>
#include <QPixmap>
#include <QGraphicsItem>
#include <QMessageBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTransform>
#include <QDebug>
#include <QVector3D>
#include <QLineEdit>
#include <QFileSystemWatcher>
#include <stack>
#include "ui_MainWindow.h"

#define ROTATION_INCREMENT 1 // 1 degree increment per signal from onRotate()

Q_DECLARE_METATYPE (std::string)

enum class SliderState {
    EDITING,
    LIVE_VIEW
};

enum class EditorState {
    MOVE,
    ROTATE
};

// Struct to store data of the applied transform between laser scans
typedef struct {
    uint32_t startIdx;
    uint32_t endIdx;
    QPointF translationOffset;
    qreal rotationOffset;
    QTransform qTransform;
} offsetState;

class MainWindow : public QMainWindow {
    Q_OBJECT
    private:
        // Default
        Ui::MainWindow* ui;

        // Widgets
        // Goal Inputs
        QLineEdit* goalXInput;
        QLineEdit* goalYInput;
        QLineEdit* goalThetaInput;
        // Lora Inputs
        QLineEdit* artifactXInput;
        QLineEdit* artifactYInput;
        QLineEdit* artifactTypeInput;

        // Robots
        int activeRobotId;

        // Console
        QFileSystemWatcher* watcher;

        CustomGraphicsScene** scenes;
        LaserScanView** currentScanScene;
        bool showGlobalMap = true;
        // XYZ values for artifact reporting and goals
        QDoubleSpinBox* artifactXBox; 
        QDoubleSpinBox* artifactYBox; 
        QDoubleSpinBox* artifactZBox;
        QDoubleSpinBox* goalXBox;
        QDoubleSpinBox* goalYBox;
        QDoubleSpinBox* goalThetaBox;
        QComboBox* comboBoxArtifactType;
        QComboBox* comboBoxGoalRobotNum;

        EStopButton** eStopButtons;


        ros::NodeHandle nh;
        Robot** robots;
        DarpaServerThread darpaServerThread;

        // TODO: Make set of variables for each Robot
        std::stack<offsetState> offsetStack;
        SliderState sliderState;
        EditorState editorState;
        uint32_t currentIndex = 0;
        double prevYaw = 0;
        QPointF prevPos;
        // End of TODO

        void initRobots();
        void initDarpaInterfaceUI();
        void initMapUI();
        void initEStopUI();
        void initGoalUI();


        QTransform getTransform(QPointF translation, double rotationAngle);
        void applyTransformList(std::vector<QGraphicsPixmapItem*> laserscans, int startIdx, 
                                int endIdx, QTransform transform, double rotationAngleTransform);
        void applyTransform(QGraphicsPixmapItem* item, 
                            QTransform transform, double rotationAngleTransform);
        QVector3D getArtifactPos();
        QVector3D getRobotGoalPos();
        std::string getArtifactType();
    public:
        MainWindow(ros::NodeHandle nh);
        ~MainWindow();
    
    signals:
        void reportArtifact(const double x, const double y, const double z, 
                                                const std::string artifactTypeStr);

    public slots:
        void selectRobot1BtnClicked();
        void selectRobot2BtnClicked();
        void selectRobot3BtnClicked();
        void selectRobot4BtnClicked();
        void selectRobot5BtnClicked();

        void reportArtifactBtnClicked();

        void addPixmap(int robotNum, const QPixmap& map, float x, float y, float theta);
        void addArtifactData(float x, float y, float z, std::string details);
        void sliderMoved(int value);
        void sendGoalBtnClicked();
        void loraDropBtnClicked();
        void propagateChanges();
        void rotatePixMap(RotateState rotateState);
        void darpaStatusRecieved(std::string teamName, double currentTime, 
                                 int32_t numReportsLeft, int32_t currentScore);
        void artifactStatusReceived(std::string result);
        void mapUpdateReceived(bool success, std::string errorStr);
        void startBtnClicked();
        void stopBtnClicked();
        void startAllBtnClicked();
        void stopAllBtnClicked();
        void switchLocalGlobal();

        void handleFileChanged(QString s);
};
#endif