//
// Created by controller on 1/11/18.
//

#ifndef PROJECT_PHOXIINTERFACE_H
#define PROJECT_PHOXIINTERFACE_H

#include <PhoXi.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#include <phoxi_camera/PhoXiException.h>

class PhoXiInterface {
public:
    PhoXiCamera();
    std::vector<std::string> cameraList();
    void connectCamera(std::string HWIdentification, pho::api::PhoXiTriggerMode mode = pho::api::PhoXiTriggerMode::Software, bool startAcquisition = true);
    void disconnectCamera();
    pho::api::PFrame getPFrame(int id = -1);
    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getPointCloud();
    static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getPointCloudFromFrame(pho::api::PFrame frame);
    void isOk();
    bool isConnected();
    bool isAcquiring();
    void startAcquisition();
    void stopAcquisition();
    int triggerImage();
    void setCoordinateSpace(pho::api::PhoXiCoordinateSpace space);
    void setTransformation(pho::api::PhoXiCoordinateTransformation coordinateTransformation,pho::api::PhoXiCoordinateSpace space,bool setSpace = true, bool saveSettings = true);
    template <typename T>
    void setTransformation(Eigen::Matrix<T,4,4> transformation,pho::api::PhoXiCoordinateSpace space,bool setSpace = true, bool saveSettings = true){
        setTransformation(getPhoXiCoordinateTransformation(transformation),space,setSpace,saveSettings);
    }
    template <typename T>
    static pho::api::PhoXiCoordinateTransformation getPhoXiCoordinateTransformation(Eigen::Matrix<T,4,4> mat){
        pho::api::PhoXiCoordinateTransformation coordinateTransformation;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                coordinateTransformation.Rotation[i][j] = mat(i,j);
            }
        }
        coordinateTransformation.Translation.x = mat(0,3);
        coordinateTransformation.Translation.y = mat(1,3);
        coordinateTransformation.Translation.z = mat(2,3);
        return coordinateTransformation;
    }
    std::string getHardwareIdentification();
    std::vector<pho::api::PhoXiCapturingMode> getSupportedCapturingModes();
    void setHighResolution();
    void setLowResolution();
    void setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition = false);
    pho::api::PhoXiTriggerMode getTriggerMode();
protected:
    pho::api::PPhoXi scanner;
private:


};


#endif //PROJECT_PHOXIINTERFACE_H
