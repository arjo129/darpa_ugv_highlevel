#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <mission_planner/RosThread.h>
#include <mission_planner/DarpaServerThread.h>
#include <mission_planner/CustomGraphicsScene.h>
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
#include <QPlainTextEdit>
#include <QCompleter>
#include <QPen>
#include <QMouseEvent>
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

        // Map
        CustomGraphicsScene* scene;

        // Widgets
        // Goal Inputs
        QLineEdit* goalXInput;
        QLineEdit* goalYInput;
        QLineEdit* goalThetaInput;
        // Lora Inputs
        QLineEdit* artifactXInput;
        QLineEdit* artifactYInput;
        QLineEdit* artifactTypeInput;
        QCompleter *completer;
        
        // Robots
        int activeRobotId;

        CustomGraphicsScene** scenes;
        LaserScanView** currentScanScene;

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

        QTransform getTransform(QPointF, double);
        void applyTransformList(std::vector<QGraphicsPixmapItem*>, int, int, QTransform, double);
        void applyTransform(QGraphicsPixmapItem*, QTransform, double);
        QVector3D getArtifactPos();
        QVector3D getRobotGoalPos();
        std::string getArtifactType();
        void selectRobot1BtnClicked();
        void selectRobot2BtnClicked();
        void selectRobot3BtnClicked();
        void selectRobot4BtnClicked();
        void selectRobot5BtnClicked();
        void handleSelectRobotBtnClicked(int);
    public:
        MainWindow(ros::NodeHandle nh);
        ~MainWindow();
    
    signals:
        void reportArtifact(const double, const double, const double, const std::string);

    public slots:
        void clickedPos(int, int);
        void reportArtifactBtnClicked();
        void addPixmap(int robotNum, const QPixmap& map, float x, float y, float theta);
        void artifactReceived(float, float, float, std::string);
        void sliderMoved(int);
        void sendGoalBtnClicked();
        void loraDropBtnClicked();
        void propagateChanges();
        void rotatePixMap(RotateState);
        void darpaStatusRecieved(std::string, double, int32_t, int32_t);
        void artifactStatusReceived(std::string);
        void rosOutReceived(std::string, std::string, std::string, int);
        void mapUpdateReceived(bool, std::string);
        void startBtnClicked();
        void stopBtnClicked();
        void startAllBtnClicked();
        void stopAllBtnClicked();
        void switchLocalGlobal();
};
#endif