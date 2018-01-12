//
// Created by controller on 1/11/18.
//

#ifndef PROJECT_PHOXIINTERFACE_H
#define PROJECT_PHOXIINTERFACE_H

#include <PhoXi.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
//#include <PhoInputOutput/Plyio.h>

class PhoXiInterface {
public:
    PhoXiCamera();
    std::vector<std::string> cameraList();
    void connectCamera(std::string HWIdentification);
    void disconnectCamera();
    pho::api::PFrame getPFrame();
    pho::api::PFrame getPFrame(int id);
    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getPointCloud();
    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getPointCloud(pho::api::PFrame frame);
    void isOk();
    bool isConnected();
    bool isAcquiring();
    void startAcquisition();
    void stopAcquisition();
    int triggerImage();
    void saveFrame(std::string path);
    void saveFrame(int id,std::string path);
    void setCoordinateSpace(pho::api::PhoXiCoordinateSpace space);
    void setTransformation(Eigen::Matrix4d transformation,pho::api::PhoXiCoordinateSpace space,bool saveSettings = true);
    std::string getHardwareIdentification();
    std::vector<pho::api::PhoXiCapturingMode> getSupportedCapturingModes(); //todo
protected:
    pho::api::PPhoXi scanner;
private:


};


#endif //PROJECT_PHOXIINTERFACE_H
