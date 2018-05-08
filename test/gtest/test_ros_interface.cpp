//
// Created by controller on 5/1/18.
//

#include "phoxi_camera/RosInterface.h"
//#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera/PhoXiException.h"
#include <phoxi_camera/GetDeviceList.h>
//#include "test_phoxi_interface.h"
#include <gtest/gtest.h>
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>

//doc
// https://github.com/google/googletest/blob/master/googletest/docs/AdvancedGuide.md

//examples
// https://github.com/google/googletest/blob/master/googletest/docs/Samples.md

// ROS test example
// https://github.com/steup/Ros-Test-Example/tree/master/src/cars
//start shutdown node
//

using namespace std;


class RosInterfaceServicesTest : public testing::Test {
public:
    ros::NodeHandle n;

    //ros service clients
    ros::ServiceClient getDeviceListService;
    ros::ServiceClient connectCameraService;
    ros::ServiceClient isConnectedService;
    ros::ServiceClient isAcquiringService;
    ros::ServiceClient isConnectedServiceV2;
    ros::ServiceClient isAcquiringServiceV2;
    ros::ServiceClient startAcquisitionService;
    ros::ServiceClient stopAcquisitionService;
    ros::ServiceClient startAcquisitionServiceV2;
    ros::ServiceClient stopAcquisitionServiceV2;
    ros::ServiceClient triggerImageService;
    ros::ServiceClient getFrameService;
    ros::ServiceClient saveFrameService;
    ros::ServiceClient disconnectCameraService;
    ros::ServiceClient getHardwareIdentificationService;
    ros::ServiceClient getSupportedCapturingModesService;
    ros::ServiceClient setCoordianteSpaceService;
    ros::ServiceClient setTransformationService;

    RosInterfaceServicesTest() {
        // create service clients
        getDeviceListService = n.serviceClient<phoxi_camera::GetDeviceList>("get_device_list");
        connectCameraService = n.serviceClient<phoxi_camera::ConnectCamera>("connect_camera");
        isConnectedService = n.serviceClient<phoxi_camera::IsConnected>("is_connected");
        isAcquiringService = n.serviceClient<phoxi_camera::IsAcquiring>("is_acquiring");
        isConnectedServiceV2= n.serviceClient<phoxi_camera::IsConnected>("V2/is_connected");
        isAcquiringServiceV2= n.serviceClient<phoxi_camera::IsAcquiring>("is_acquiring");
        startAcquisitionService= n.serviceClient<std_srvs::Empty>("start_acquisition");
        stopAcquisitionService= n.serviceClient<std_srvs::Empty>("stop_acquisition");
        startAcquisitionServiceV2= n.serviceClient<phoxi_camera::Empty>("V2/start_acquisition");
        stopAcquisitionServiceV2= n.serviceClient<phoxi_camera::Empty>("V2/stop_acquisition");
        triggerImageService = n.serviceClient<phoxi_camera::TriggerImage>("trigger_image");
        getFrameService = n.serviceClient<phoxi_camera::GetFrame>("get_frame");
        saveFrameService = n.serviceClient<phoxi_camera::SaveFrame>("save_frame");
        disconnectCameraService = n.serviceClient<std_srvs::Empty>("disconnect_camera");
        getHardwareIdentificationService = n.serviceClient<phoxi_camera::GetHardwareIdentification>("get_hardware_indentification");
        getSupportedCapturingModesService = n.serviceClient<phoxi_camera::GetSupportedCapturingModes>("get_supported_capturing_modes");
        setCoordianteSpaceService = n.serviceClient<phoxi_camera::SetCoordinatesSpace>("V2/set_transformation");
        setTransformationService = n.serviceClient<phoxi_camera::SetTransformationMatrix>("V2/set_coordination_space");
    }
    static void SetUpTestCase() {
        //test constructor
        ASSERT_NO_THROW(RosInterface());
    }
};

class DISABLED_ROSInterfaceTest : public testing::Test {
public:
    static void SetUpTestCase(){}
};


TEST_F (RosInterfaceServicesTest, getDeviceList) {
    RosInterface ros_interface;
    phoxi_camera::GetDeviceList srv;

    ASSERT_EQ(getDeviceListService.call(srv), true);
    ASSERT_EQ(srv.response.success, true);
    ASSERT_EQ(srv.response.message, "Ok");
}



//
//TEST_F (ROSInterfaceTest, connect_with_acquisition) {
//    PhoXiInterface phoxi_interface = PhoXiInterface();
//
//
//}
//
//TEST_F (DISABLED_ROSInterfaceTest, disconnect) {
//    PhoXiInterface phoxi_interface = PhoXiInterface();
//
//}
//
//TEST_F (DISABLED_ROSInterfaceTest, isOK) {
//    PhoXiInterface phoxi_interface = PhoXiInterface();
//
//}



