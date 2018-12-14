//
// Created by controller on 1/11/18.
//

#include "phoxi_camera/PhoXiInterface.h"
namespace phoxi_camera {
    PhoXiInterface::PhoXiInterface() {

    }

    std::vector<std::string> PhoXiInterface::cameraList() {
        if (!phoXiFactory.isPhoXiControlRunning()) {
            scanner.Reset();
            throw PhoXiControlNotRunning("PhoXi Control is not running");
        }
        std::vector<std::string> list;
        auto DeviceList = phoXiFactory.GetDeviceList();
        for (int i = 0; i < DeviceList.size(); ++i) {
            list.push_back(DeviceList[i].HWIdentification);
        }
        return list;
    }

    void PhoXiInterface::connectCamera(std::string HWIdentification, pho::api::PhoXiTriggerMode mode,
                                       bool startAcquisition) {
        if (this->isConnected()) {
            if (scanner->HardwareIdentification == HWIdentification) {
                this->setTriggerMode(mode, startAcquisition);
                return;
            }
        }
        if (!phoXiFactory.isPhoXiControlRunning()) {
            throw PhoXiControlNotRunning("PhoXi Control is not running");
        }
        auto DeviceList = phoXiFactory.GetDeviceList();
        bool found = false;
        std::string device;
        for (int i = 0; i < DeviceList.size(); i++) {
            if (DeviceList[i].HWIdentification == HWIdentification) {
                found = true;
                device = DeviceList[i].HWIdentification;
                break;
            }
        }
        if (!found) {
            throw PhoXiScannerNotFound("Scanner not found");
        }
        disconnectCamera();
        if (!(scanner = phoXiFactory.CreateAndConnect(device, 5000))) {
            disconnectCamera();
            throw UnableToStartAcquisition("Scanner was not able to connect. Disconnected.");
        }
        this->setTriggerMode(mode, startAcquisition);
    }

    void PhoXiInterface::disconnectCamera() {
        if (scanner && scanner->isConnected()) {
            scanner->Disconnect(true);
        }
        last_frame_id = -1;
    }

    pho::api::PFrame PhoXiInterface::getPFrame(int id) {
        if (id < 0) {
            try {
                id = this->triggerImage();
            } catch (UnableToTriggerFrame &e) {
                throw;
            }

        }
        this->isOk();
        return scanner->GetSpecificFrame(id, 10000);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getPointCloud(bool organized) {
        return getPointCloudFromFrame(getPFrame(-1), organized);
    }

    static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getPointCloudFromFrame(pho::api::PFrame frame, bool organized) {
        if(organized){
            return getOrganizedCloudFromFrame(frame);
        }
        return getUnorganizedCloudFromFrame(frame);
    }
    static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getOrganizedCloudFromFrame(pho::api::PFrame frame){
        if (!frame || !frame->Successful) {
            throw CorruptedFrame("Corrupted frame!");
        }
        std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> cloud(new pcl::PointCloud<pcl::PointNormal>(frame->GetResolution().Width, frame->GetResolution().Height));
        for (int r = 0; r < frame->GetResolution().Height; r++) {
            for (int c = 0; c < frame->GetResolution().Width; c++) {
                auto point = frame->PointCloud.At(r, c);
                pcl::PointNormal pclPoint;
                pclPoint.x = point.x / 1000;                 //to [m]
                pclPoint.y = point.y / 1000;                 //to [m]
                pclPoint.z = point.z / 1000;                 //to [m]
                if (!frame->NormalMap.Empty()) {
                    auto normal = frame->NormalMap.At(r, c);
                    pclPoint.normal_x = normal.x / 1000;    //to [m]
                    pclPoint.normal_y = normal.y / 1000;    //to [m]
                    pclPoint.normal_z = normal.z / 1000;    //to [m]
                }
                cloud->at(c, r) = pclPoint;
            }
        }
        return cloud;
    }

    static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getUnorganizedCloudFromFrame(pho::api::PFrame frame){
        if (!frame || !frame->Successful) {
            throw CorruptedFrame("Corrupted frame!");
        }
        std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> cloud(new pcl::PointCloud<pcl::PointNormal>());
        for (int r = 0; r < frame->GetResolution().Height; r++) {
            for (int c = 0; c < frame->GetResolution().Width; c++) {
                auto point = frame->PointCloud.At(r, c);
                if(point == pho::api::Point3_32f(0,0,0)){
                    continue;
                }
                pcl::PointNormal pclPoint;
                pclPoint.x = point.x / 1000;                 //to [m]
                pclPoint.y = point.y / 1000;                 //to [m]
                pclPoint.z = point.z / 1000;                 //to [m]
                if (!frame->NormalMap.Empty()) {
                    auto normal = frame->NormalMap.At(r, c);
                    pclPoint.normal_x = normal.x / 1000;    //to [m]
                    pclPoint.normal_y = normal.y / 1000;    //to [m]
                    pclPoint.normal_z = normal.z / 1000;    //to [m]
                }
                cloud->push_back(pclPoint);
            }
        }
        return cloud;
    }

    void PhoXiInterface::isOk() {
        if (!scanner || !scanner->isConnected()) {
            throw PhoXiScannerNotConnected("No scanner connected");
        }
    }

    std::string PhoXiInterface::getHardwareIdentification() {
        this->isOk();
        return scanner->HardwareIdentification;
    }

    bool PhoXiInterface::isConnected() {
        return (scanner && scanner->isConnected());
    }

    bool PhoXiInterface::isAcquiring() {
        return (scanner && scanner->isAcquiring());
    }

    void PhoXiInterface::startAcquisition() {
        this->isOk();
        if (scanner->isAcquiring()) {
            return;
        }
        scanner->StartAcquisition();
        if (!scanner->isAcquiring()) {
            throw UnableToStartAcquisition("Unable to start acquisition.");
        }
    }

    void PhoXiInterface::stopAcquisition() {
        this->isOk();
        if (!scanner->isAcquiring()) {
            return;
        }
        scanner->StopAcquisition();
        if (scanner->isAcquiring()) {
            throw UnableToStopAcquisition("Unable to stop acquisition.");
        }
    }

    int PhoXiInterface::triggerImage() {
        this->setTriggerMode(pho::api::PhoXiTriggerMode::Software, true);
        int frame_id = scanner->TriggerFrame();
        last_frame_id = frame_id;

        if (frame_id < 0) {
            switch (frame_id) {
                case -1:
                    throw UnableToTriggerFrame("Trigger not accepted.");
                case -2:
                    throw UnableToTriggerFrame("Device is not running.");
                case -3:
                    throw UnableToTriggerFrame("Communication Error.");
                case -4:
                    throw UnableToTriggerFrame("WaitForGrabbingEnd is not supported.");
                default:
                    throw UnableToTriggerFrame("Unknown error.");
            }
        }
        return frame_id;
    }

    std::vector<pho::api::PhoXiCapturingMode> PhoXiInterface::getSupportedCapturingModes() {
        this->isOk();
        return scanner->SupportedCapturingModes;
    }

    void PhoXiInterface::setHighResolution() {
        this->isOk();
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        mode.Resolution.Width = 2064;
        mode.Resolution.Height = 1544;
        scanner->CapturingMode = mode;
    }

    void PhoXiInterface::setLowResolution() {
        this->isOk();
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        mode.Resolution.Width = 1032;
        mode.Resolution.Height = 772;
        scanner->CapturingMode = mode;
    }

    void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition) {
        if (!((mode == pho::api::PhoXiTriggerMode::Software) || (mode == pho::api::PhoXiTriggerMode::Hardware) ||
              (mode == pho::api::PhoXiTriggerMode::Freerun) || (mode == pho::api::PhoXiTriggerMode::NoValue))) {
            throw InvalidTriggerMode("Invalid trigger mode " + std::to_string(mode) + ".");
        }
        this->isOk();
        if (mode == scanner->TriggerMode.GetValue()) {
            if (startAcquisition) {
                this->startAcquisition();
            } else {
                this->stopAcquisition();
            }
            return;
        }
        this->stopAcquisition();
        scanner->TriggerMode = mode;
        if (startAcquisition) {
            this->startAcquisition();
        }
    }

    pho::api::PhoXiTriggerMode PhoXiInterface::getTriggerMode() {
        this->isOk();
        return scanner->TriggerMode;
    }

#ifndef PHOXI_API_v1_1
    void PhoXiInterface::setCoordinateSpace(pho::api::PhoXiCoordinateSpace space) {
        this->isOk();
        scanner->CoordinatesSettings->CoordinateSpace = space;
    }

    pho::api::PhoXiCoordinateSpace PhoXiInterface::getCoordinateSpace() {
        this->isOk();
        return scanner->CoordinatesSettings->CoordinateSpace;
    }

    void PhoXiInterface::setTransformation(pho::api::PhoXiCoordinateTransformation coordinateTransformation,
                                           pho::api::PhoXiCoordinateSpace space, bool setSpace = true,
                                           bool saveSettings = true) {
        this->isOk();
        pho::api::PhoXiCoordinatesSettings settings = scanner->CoordinatesSettings;
        switch (space) {
            case pho::api::PhoXiCoordinateSpace::RobotSpace:
                if (!settings.RobotTransformation.isSupported()) {
                    throw CoordinateSpaceNotSupported("Coordinate space is not supported");
                }
                settings.RobotTransformation = coordinateTransformation;
                settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::RobotSpace;
                break;
            case pho::api::PhoXiCoordinateSpace::CustomSpace:
                if (!settings.CustomTransformation.isSupported()) {
                    throw CoordinateSpaceNotSupported("Coordinate space is not supported");
                }
                settings.CustomTransformation = coordinateTransformation;
                settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::CustomSpace;
                break;
            default:
                throw CoordinateSpaceNotSupported("Coordinate space is not supported");
        }
        if (setSpace) {
            settings.CoordinateSpace = space;
        }
        this->isOk();
        scanner->CoordinatesSettings = settings;
        if (saveSettings) {
            scanner->SaveSettings();
        }
    }

    bool PhoXiInterface::saveLastFrame(const std::string &path) {
        if (last_frame_id == -1) {
            // this is needed due to PhoXi API bug
            // if you didn't trigger frame before, you can not save it
            return false;
        }

        bool success = scanner->SaveLastOutput(path, this->last_frame_id);
        return success;
    }
#endif

}
