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
        throw std::runtime_error("PhoXi Controll is not running");
    }
    std::vector<std::string> list;
    auto DeviceList = phoXiFactory.GetDeviceList();
    for (int i = 0; i < DeviceList.size(); ++i) {
        list.push_back(DeviceList[i].HWIdentification);
    }
    return list;
}

void PhoXiInterface::connectCamera(std::string HWIdentification){
    pho::api::PhoXiFactory phoXiFactory;
    if(scanner && scanner->isConnected()){
        if(scanner->HardwareIdentification == HWIdentification){
            return;
        }
    }
    if (!phoXiFactory.isPhoXiControlRunning()) {
        throw std::runtime_error("PhoXi Controll is not running");
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
        throw std::runtime_error("Scanner not found");
    }
    disconnectCamera();
    if(!(scanner = phoXiFactory.CreateAndConnect(device,5000))){
        disconnectCamera();
        throw std::runtime_error("Scanner was not able to connect. Disconnected.");
    }
    scanner->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    scanner->StartAcquisition();
    if(!scanner->isAcquiring()){
        disconnectCamera();
        throw std::runtime_error("Scanner was not able to acquire. Disconnected.");
    }
    return ;
}
void PhoXiInterface::disconnectCamera(){
    this->isOk();
    scanner->Disconnect();
}

pho::api::PFrame PhoXiInterface::getPFrame(){
    this->isOk();
    return  scanner->GetSpecificFrame(scanner->TriggerFrame());
}
pho::api::PFrame PhoXiInterface::getPFrame(int id){
    this->isOk();
    return  scanner->GetSpecificFrame(id);
}

std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getPointCloud() {
    pho::api::PFrame frame = getPFrame();
    if (!frame || !frame->Successful) {
        throw std::runtime_error("Bad frame arrived");
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
            pclPoint.getVector3fMap() = eigenPoint;
            pclPoint.getNormalVector3fMap() = eigenNormal;
            cloud->at(c,r) = pclPoint;
        }
    }
    return cloud;
}

void PhoXiInterface::isOk(){
    if(!scanner || !scanner->isConnected()){
        throw std::runtime_error("No scanner connected");
    }
    return;
}

void PhoXiInterface::setCoordinateSpace(pho::api::PhoXiCoordinateSpace space){
    this->isOk();
    scanner->CoordinatesSettings->CoordinateSpace = space;
    return;
}

void PhoXiInterface::setTransformation(Eigen::Matrix4d transformation, pho::api::PhoXiCoordinateSpace space, bool saveSettings) {
    this->isOk();
    pho::api::PhoXiCoordinateTransformation coordinateTransformation;
    pho::api::PhoXiCoordinatesSettings settings = scanner->CoordinatesSettings;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            coordinateTransformation.Rotation[i][j] = transformation(i,j);
        }
    }
    coordinateTransformation.Translation.x = transformation(0,3);
    coordinateTransformation.Translation.y = transformation(1,3);
    coordinateTransformation.Translation.z = transformation(2,3);
    switch(space){
        case pho::api::PhoXiCoordinateSpace::RobotSpace:
            if(!settings.RobotTransformation.isSupported()){
                throw std::runtime_error("Coordination space is not supported");
            }
            settings.RobotTransformation = coordinateTransformation;
            settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::RobotSpace;
            break;
        case pho::api::PhoXiCoordinateSpace::CustomSpace:
            if(!settings.CustomTransformation.isSupported()){
                throw std::runtime_error("Coordination space is not supported");
            }
            settings.CustomTransformation = coordinateTransformation;
            settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::CustomSpace;
            break;
        default:
            throw std::runtime_error("Coordination space is not supported");
    }

    this->isOk();
    scanner->CoordinatesSettings = settings;
    if(saveSettings){
        scanner->SaveSettings();
    }
    return;
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
    scanner->StartAcquisition();
}
void PhoXiInterface::stopAcquisition(){
    this->isOk();
    scanner->StopAcquisition();
}
int PhoXiInterface::triggerImage(){
    this->isOk();
    return scanner->TriggerFrame();
}
void PhoXiInterface::saveFrame(int id, std::string path){
    this->isOk();
    pho::api::PFrame frame = scanner->GetSpecificFrame(id);
    frame->SaveAsPly(path);
}
void PhoXiInterface::saveFrame(std::string path){
    this->isOk();
    pho::api::PFrame frame = scanner->GetSpecificFrame(scanner->TriggerFrame());
    frame->SaveAsPly(path);
}
std::vector<pho::api::PhoXiCapturingMode> PhoXiInterface::getSupportedCapturingModes(){
    this->isOk();
    return scanner->SupportedCapturingModes;
}