#ifndef _DARPA_SERVER_THREAD_H_
#define _DARPA_SERVER_THREAD_H_

#include <ros/ros.h>
#include <interface_protocol/GetStatus.h>
#include <interface_protocol/GetStatusRequest.h>
#include <interface_protocol/GetStatusResponse.h>
#include <interface_protocol/MappingUpdate.h>
#include <interface_protocol/MappingUpdateRequest.h>
#include <interface_protocol/MappingUpdateResponse.h>
#include <interface_protocol/PostReport.h>
#include <interface_protocol/PostReportRequest.h>
#include <interface_protocol/PostReportResponse.h>
#include <QThread>

class DarpaServerThread : public QThread {
    
    Q_OBJECT
    protected:
        void run() override;
    public: 
        DarpaServerThread(ros::NodeHandle nh);
        ~DarpaServerThread();
        bool initInterfaceProtocolServices();
        
    private:
        bool isRunning;
        ros::NodeHandle nh;
        ros::ServiceClient artifactReportSrvClient, darpaStatusSrvClient, mapReportSrvClient,
                           telemetryReportSrvClient, markersReportSrvClient;

    signals:
        void darpaStatusRecieved(std::string teamName, double currentTime, 
                                 int32_t numReportsLeft, int32_t currentScore);
        void artifactStatusReceived(std::string result);
        void mapUpdateReceived(bool success, std::string errorStr);

    public slots:
        void getDarpaStatus();
        void reportArtifact(const double x, const double y, const double z, 
                                                const std::string artifactTypeStr);
        void reportMapPoseMarkers();

};



#endif /* _DARPA_SERVER_THREAD_H_ */