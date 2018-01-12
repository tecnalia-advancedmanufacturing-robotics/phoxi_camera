//
// Created by controller on 1/11/18.
//

#ifndef PROJECT_ROSINTERFACE_H
#define PROJECT_ROSINTERFACE_H

#include <ros/ros.h>
#include <phoxi_camera/PhoXiInterface.h>
#include <phoxi_camera/GetDeviceList.h>
#include <phoxi_camera/ConnectCamera.h>
#include <phoxi_camera/IsConnected.h>
#include <phoxi_camera/IsAcquiring.h>
#include <phoxi_camera/Bool.h>
#include <std_srvs/Empty.h>
#include <phoxi_camera/Empty.h>
#include <phoxi_camera/TriggerImage.h>
#include <phoxi_camera/GetFrame.h>
#include <phoxi_camera/SaveFrame.h>
#include <phoxi_camera/GetHardwareIdentification.h>
#include <phoxi_camera/GetSupportedCapturingModes.h>
#include <phoxi_camera/SetCoordinatesSpace.h>
#include <phoxi_camera/TransformationMatrix.h>


class RosInterface : protected  PhoXiInterface {
public:
    RosInterface();
protected:
    void publishFrame(pho::api::PFrame frame);

    std::string parrent_frame, child_frame; //todo default initialization
private:
    bool getDeviceList(phoxi_camera::GetDeviceList::Request &req, phoxi_camera::GetDeviceList::Response &res);
    bool connectCamera(phoxi_camera::ConnectCamera::Request &req, phoxi_camera::ConnectCamera::Response &res);
    bool isConnected(phoxi_camera::IsConnected::Request &req, phoxi_camera::IsConnected::Response &res);
    bool isAcquiring(phoxi_camera::IsAcquiring::Request &req, phoxi_camera::IsAcquiring::Response &res);
    bool isConnected(phoxi_camera::Bool::Request &req, phoxi_camera::Bool::Response &res);
    bool isAcquiring(phoxi_camera::Bool::Request &req, phoxi_camera::Bool::Response &res);
    bool startAcquisition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool stopAcquisition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool startAcquisition(phoxi_camera::Empty::Request &req, phoxi_camera::Empty::Response &res);
    bool stopAcquisition(phoxi_camera::Empty::Request &req, phoxi_camera::Empty::Response &res);
    bool triggerImage(phoxi_camera::TriggerImage::Request &req, phoxi_camera::TriggerImage::Response &res);
    bool getFrame(phoxi_camera::GetFrame::Request &req, phoxi_camera::GetFrame::Response &res);
    bool saveFrame(phoxi_camera::SaveFrame::Request &req, phoxi_camera::SaveFrame::Response &res);
    bool disconnectCamera(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool getHardwareIdentification(phoxi_camera::GetHardwareIdentification::Request &req, phoxi_camera::GetHardwareIdentification::Response &res);
    bool getSupportedCapturingModes(phoxi_camera::GetSupportedCapturingModes::Request &req, phoxi_camera::GetSupportedCapturingModes::Response &res);
    bool setCoordianteSpace(phoxi_camera::SetCoordinatesSpace::Request &req, phoxi_camera::SetCoordinatesSpace::Response &res);
    bool setTransformation(phoxi_camera::TransformationMatrix::Request &req, phoxi_camera::TransformationMatrix::Response &res);
    //node handle
    ros::NodeHandle nh;

    //ros service servers
    ros::ServiceServer getDeviceListService;
    ros::ServiceServer connectCameraService;
    ros::ServiceServer isConnectedService;
    ros::ServiceServer isAcquiringService;
    ros::ServiceServer isAcquiringServiceV2;
    ros::ServiceServer isConnectedServiceV2;
    ros::ServiceServer startAcquisitionService;
    ros::ServiceServer stopAcquisitionService;
    ros::ServiceServer startAcquisitionServiceV2;
    ros::ServiceServer stopAcquisitionServiceV2;
    ros::ServiceServer triggerImageService;
    ros::ServiceServer getFrameService;
    ros::ServiceServer saveFrameService;
    ros::ServiceServer disconnectCameraService;
    ros::ServiceServer getHardwareIdentificationService;
    ros::ServiceServer getSupportedCapturingModesService;
    ros::ServiceServer setCoordianteSpaceService;
    ros::ServiceServer setTransformationService;

    //ros publishers
    ros::Publisher cloudPub;
    ros::Publisher normalMapPub;
    ros::Publisher confidenceMapPub;
    ros::Publisher texturePub;
};


#endif //PROJECT_ROSINTERFACE_H
