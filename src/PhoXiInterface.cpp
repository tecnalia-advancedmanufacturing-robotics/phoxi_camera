//
// Created by controller on 1/11/18.
//

#include "phoxi_camera/PhoXiInterface.h"

PhoXiInterface::PhoXiCamera(){

}

std::vector<std::string> PhoXiInterface::cameraList(){
    pho::api::PhoXiFactory phoXiFactory;
    if (!phoXiFactory.isPhoXiControlRunning()){
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

void PhoXiInterface::connectCamera(std::string HWIdentification, pho::api::PhoXiTriggerMode mode, bool startAcquisition){
    pho::api::PhoXiFactory phoXiFactory;
    if(scanner && scanner->isConnected()){
        if(scanner->HardwareIdentification == HWIdentification){
            return;
        }
    }
    if (!phoXiFactory.isPhoXiControlRunning()) {
        throw PhoXiControlNotRunning("PhoXi Control is not running");
    }
    auto DeviceList = phoXiFactory.GetDeviceList();
    bool found = false;
    std::string device;
    for(int i= 0; i< DeviceList.size(); i++){
        if(DeviceList[i].HWIdentification == HWIdentification) {
            found = true;
            device = DeviceList[i].HWIdentification;
            break;
        }
    }
    if(!found){
        throw PhoXiScannerNotFound("Scanner not found");
    }
    disconnectCamera();
    if(!(scanner = phoXiFactory.CreateAndConnect(device,5000))){
        disconnectCamera();
        throw UnableToStartAcquisition("Scanner was not able to connect. Disconnected.");
    }
    this->setTriggerMode(mode,startAcquisition);
    return ;
}
void PhoXiInterface::disconnectCamera(){
    if(scanner && scanner->isConnected()){
        scanner->Disconnect(true);
    }
}

pho::api::PFrame PhoXiInterface::getPFrame(int id){
    if(id < 0){
        id = this->triggerImage();
    }
    this->isOk();
    return  scanner->GetSpecificFrame(id,10000);
}

std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getPointCloud() {
    return getPointCloudFromFrame(getPFrame(-1)); //todo
}

static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getPointCloudFromFrame(pho::api::PFrame frame) {
    if (!frame || !frame->Successful) {
        throw CorruptedFrame("Corrupted frame!");
    }
    std::shared_ptr <pcl::PointCloud<pcl::PointNormal>>cloud(new pcl::PointCloud<pcl::PointNormal>(frame->GetResolution().Width,frame->GetResolution().Height));
    for(int r = 0; r < frame->GetResolution().Height; r++){
        for (int c = 0; c < frame->GetResolution().Width; c++){
            //todo
            auto point = frame->PointCloud.At(r,c);
            auto normal = frame->NormalMap.At(r,c);
            Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
            Eigen::Vector3f eigenNormal(point.x, point.y, point.z);
            pcl::PointNormal pclPoint;
            pclPoint.getVector3fMap() = eigenPoint/1000;  //to [m]
            pclPoint.getNormalVector3fMap() = eigenNormal/1000; //to [m]
            cloud->at(c,r) = pclPoint;
        }
    }
    return cloud;
}

void PhoXiInterface::isOk(){
    if(!scanner || !scanner->isConnected()){
        throw PhoXiScannerNotConnected("No scanner connected");
    }
    return;
}

void PhoXiInterface::setCoordinateSpace(pho::api::PhoXiCoordinateSpace space){
    this->isOk();
    scanner->CoordinatesSettings->CoordinateSpace = space;
    return;
}
void PhoXiInterface::setTransformation(pho::api::PhoXiCoordinateTransformation coordinateTransformation,pho::api::PhoXiCoordinateSpace space,bool setSpace = true, bool saveSettings = true){
    this->isOk();
    pho::api::PhoXiCoordinatesSettings settings = scanner->CoordinatesSettings;
    switch(space){
        case pho::api::PhoXiCoordinateSpace::RobotSpace:
            if(!settings.RobotTransformation.isSupported()){
                throw CoordinationSpaceNotSupported("Coordination space is not supported");
            }
            settings.RobotTransformation = coordinateTransformation;
            settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::RobotSpace;
            break;
        case pho::api::PhoXiCoordinateSpace::CustomSpace:
            if(!settings.CustomTransformation.isSupported()){
                throw CoordinationSpaceNotSupported("Coordination space is not supported");
            }
            settings.CustomTransformation = coordinateTransformation;
            settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::CustomSpace;
            break;
        default:
            throw CoordinationSpaceNotSupported("Coordination space is not supported");
    }
    if(setSpace){
        settings.CoordinateSpace = space;
    }
    this->isOk();
    scanner->CoordinatesSettings = settings;
    if(saveSettings){
        scanner->SaveSettings();
    }
}

std::string PhoXiInterface::getHardwareIdentification(){
    this->isOk();
    return scanner->HardwareIdentification;
}

bool PhoXiInterface::isConnected(){
    return (scanner && scanner->isConnected());
}
bool PhoXiInterface::isAcquiring(){
    return (scanner && scanner->isAcquiring());
}
void PhoXiInterface::startAcquisition(){
    this->isOk();
    if(scanner->isAcquiring()){
        return;
    }
    scanner->StartAcquisition();
    if(!scanner->isAcquiring()){
        throw UnableToStartAcquisition("Unable to start acquisition.");
    }
}
void PhoXiInterface::stopAcquisition(){
    this->isOk();
    if(!scanner->isAcquiring()){
        return;
    }
    scanner->StopAcquisition();
    if(scanner->isAcquiring()){
        throw UnableToStopAcquisition("Unable to stop acquisition.");
    }
}
int PhoXiInterface::triggerImage(){
    this->isOk();
    if(scanner->TriggerMode != pho::api::PhoXiTriggerMode::Software){
        this->setTriggerMode(pho::api::PhoXiTriggerMode::Software,true);
    }
    return scanner->TriggerFrame();
}

std::vector<pho::api::PhoXiCapturingMode> PhoXiInterface::getSupportedCapturingModes(){
    this->isOk();
    return scanner->SupportedCapturingModes;
}

void PhoXiInterface::setHighResolution(){
    this->isOk();
    pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
    mode.Resolution.Width = 2064;
    mode.Resolution.Height = 1544;
    scanner->CapturingMode = mode;
}
void PhoXiInterface::setLowResolution(){
    this->isOk();
    pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
    mode.Resolution.Width = 1032;
    mode.Resolution.Height = 772;
    scanner->CapturingMode = mode;
}

void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition){
    this->isOk();
    if(mode == scanner->TriggerMode.GetValue()){
        if(startAcquisition){
            this->startAcquisition();
        }
        else{
            this->stopAcquisition();
        }
        return;
    }
    this->stopAcquisition();
    scanner->TriggerMode = mode;
    if(startAcquisition){
        this->startAcquisition();
    }
}
