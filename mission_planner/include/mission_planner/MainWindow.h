#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <mission_planner/RosThread.h>
#include <mission_planner/DarpaServerThread.h>
#include <mission_planner/CustomGraphicsScene.h>
#include <QPixmap>
#include <QGraphicsItem>
#include <QMessageBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTransform>
#include <QDebug>
#include <QVector3D>
#include <stack>
#include "ui_MainWindow.h"

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
        Ui::MainWindow* ui;
        CustomGraphicsScene* scene;
        // XYZ values for artifact reporting and goals
        QDoubleSpinBox* artifactXBox; 
        QDoubleSpinBox* artifactYBox; 
        QDoubleSpinBox* artifactZBox;
        QDoubleSpinBox* goalXBox;
        QDoubleSpinBox* goalYBox;
        QDoubleSpinBox* goalZBox;
        QComboBox* comboBoxArtifactType;


        ros::NodeHandle nh;
        ROSThread rosthread;
        DarpaServerThread darpaServerThread;
        std::vector<QGraphicsPixmapItem*> laserscans;
        std::stack<offsetState> offsetStack;
        SliderState sliderState;
        EditorState editorState;
        uint32_t currentIndex = 0;
        double prevYaw = 0;
        QPointF prevPos;

        QTransform getTransform(QPointF translation, double rotationAngle);
        void applyTransformList(int startIdx, int endIdx, 
                                QTransform transform, double rotationAngleTransform);
        void applyTransform(QGraphicsPixmapItem* item, 
                            QTransform transform, double rotationAngleTransform);
        
        void initDarpaInterfaceUI();
        QVector3D getArtifactPos();
        std::string getArtifactTypeStr();
    public:
        MainWindow(ros::NodeHandle nh);
        ~MainWindow();
    
    signals:
        void reportArtifact(const double x, const double y, const double z, 
                                                const std::string artifactTypeStr);

    public slots:
        void addPixmap(const QPixmap& map, int x, int y, float theta);
        void sliderMoved(int value);
        void artifactBtnClicked();
        void propagateChanges();
        void rotatePixMap(RotateState rotateState);
        void darpaStatusRecieved(std::string teamName, double currentTime, 
                                 int32_t numReportsLeft, int32_t currentScore);
        void artifactStatusReceived(std::string result);
        void mapUpdateReceived(bool success, std::string errorStr);
};
#endif