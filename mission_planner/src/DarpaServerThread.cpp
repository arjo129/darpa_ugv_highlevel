#include <mission_planner/DarpaServerThread.h>

DarpaServerThread::DarpaServerThread(ros::NodeHandle _nh) : nh(_nh) {
    initInterfaceProtocolServices();
    this->isRunning = true;
}

DarpaServerThread::~DarpaServerThread() {
    this->isRunning = false;
}

bool DarpaServerThread::initInterfaceProtocolServices() {
    this->artifactReportSrvClient = this->nh.serviceClient<interface_protocol::PostReport>("post_report");
    this->darpaStatusSrvClient = this->nh.serviceClient<interface_protocol::GetStatus>("get_status");
    this->mapReportSrvClient = this->nh.serviceClient<interface_protocol::MappingUpdate>("post_map_update");
    this->telemetryReportSrvClient = this->nh.serviceClient<interface_protocol::MappingUpdate>("post_telemetry_update");
    this->markersReportSrvClient = this->nh.serviceClient<interface_protocol::MappingUpdate>("post_markers_update");

    while (ros::ok()) {
        if (!(this->artifactReportSrvClient.exists() && this->darpaStatusSrvClient.exists() && this->mapReportSrvClient.exists() && 
            this->telemetryReportSrvClient.exists() && this->markersReportSrvClient.exists())) {
            ROS_ERROR("One or more of the interface_protocol services are down. Please Check.");
            ROS_ERROR("Trying Again...");
            ros::Duration(2.0).sleep();
        }
        else {
            ROS_INFO("Successfully connected to interface_protocol services");
            break;
        }
    }
    
}

void DarpaServerThread::getDarpaStatus() {
    interface_protocol::GetStatusRequest getStatusRequest;
    interface_protocol::GetStatusResponse getStatusResponse;
    if (this->darpaStatusSrvClient.call(getStatusRequest, getStatusResponse)) {
        emit darpaStatusRecieved(getStatusResponse.current_team, getStatusResponse.run_clock, 
                                 getStatusResponse.remaining_reports, getStatusResponse.score);
    }
    else {
        emit darpaStatusRecieved("ROS Service Error", -1, -1, -1);
    }
}

void DarpaServerThread::reportArtifact(const double x, const double y, const double z, 
                                                    const std::string artifactTypeStr) {
    // The commented part seems to be blocking the GUI....
    // while (!this->artifactReportSrvClient.exists()) {
    //     ros::Duration(2.0).sleep();
    //     ROS_ERROR("Artifact Service post_report is down on interface_protocol. Trying again...");
    // }

    interface_protocol::PostReportRequest postReportRequest;
    postReportRequest.x = x;
    postReportRequest.y = y;
    postReportRequest.z = z;
    postReportRequest.type = artifactTypeStr;

    interface_protocol::PostReportResponse postReportResponse;
    if (this->artifactReportSrvClient.call(postReportRequest, postReportResponse)) {
        emit artifactStatusReceived(postReportResponse.report_status);
    }
    else {
         emit artifactStatusReceived("Artifact Service post_report is down on interface_protocol.");
    }
    
}

void DarpaServerThread::reportMapPoseMarkers() {

    interface_protocol::MappingUpdateRequest mappingUpdateRequest;
    interface_protocol::MappingUpdateResponse mappingUpdateResponse_1, mappingUpdateResponse_2, mappingUpdateResponse_3;
    this->mapReportSrvClient.call(mappingUpdateRequest, mappingUpdateResponse_1);
    this->telemetryReportSrvClient.call(mappingUpdateRequest, mappingUpdateResponse_2);
    this->markersReportSrvClient.call(mappingUpdateRequest, mappingUpdateResponse_3);

    if (mappingUpdateResponse_1.success && mappingUpdateResponse_2.success && 
                                                mappingUpdateResponse_3.success) {
        emit mapUpdateReceived(true, "Success");
    }
    else {
        emit mapUpdateReceived(false, mappingUpdateResponse_1.result + 
                                mappingUpdateResponse_2.result + mappingUpdateResponse_3.result);
    }

    
}

void DarpaServerThread::run() {
    ros::Rate rate(10);
    while(this->isRunning) {
        ros::spinOnce();
        rate.sleep();
    }
}