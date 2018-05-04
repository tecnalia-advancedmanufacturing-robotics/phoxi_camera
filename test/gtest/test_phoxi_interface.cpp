//
// Created by Jakub Hazik on 4/18/18.
//

#include <gtest/gtest.h>
#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera/PhoXiException.h"
#include "PhoXi.h"

#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>

using namespace std;

class PhoXiInterfaceTest : public testing::Test {
public:
    const string camera_ID = "InstalledExamples-PhoXi-example";
    PhoXiInterface phoxi_interface;

    static void SetUpTestCase(){
        //test constructor only once
        ASSERT_NO_THROW(PhoXiInterface());
    }

    virtual void SetUp() {
        phoxi_interface.connectCamera(camera_ID);
    }

    virtual void TearDown() {
        phoxi_interface.disconnectCamera();
    }
};

class PhoXiInterfaceTestConnection : public testing::Test {
public:
    const string camera_ID = "InstalledExamples-PhoXi-example";
    PhoXiInterface phoxi_interface;

    static void SetUpTestCase(){
        //test constructor
        ASSERT_NO_THROW(PhoXiInterface());
    }

    virtual void TearDown() {
        this->phoxi_interface.disconnectCamera();
    }
};

TEST_F (PhoXiInterfaceTestConnection, connect) {
    PhoXiInterface phoxi_interface = PhoXiInterface();

    string incorrect_camera_ID = "000000";

    // try to connect to unreachable scanner
    ASSERT_THROW(phoxi_interface.connectCamera(incorrect_camera_ID), PhoXiScannerNotFound);
    ASSERT_FALSE(phoxi_interface.isConnected());
    EXPECT_THROW(phoxi_interface.isOk(), PhoXiScannerNotConnected);

    //  try to connect to reachable scanner
    ASSERT_NO_THROW(phoxi_interface.connectCamera(camera_ID));
    ASSERT_TRUE(phoxi_interface.isConnected());
    ASSERT_NO_THROW(phoxi_interface.isOk());

    // try connect to camera when I am already connected
    ASSERT_NO_THROW(phoxi_interface.connectCamera(camera_ID));
    ASSERT_TRUE(phoxi_interface.isConnected());
    ASSERT_NO_THROW(phoxi_interface.isOk());

    // I am connected to scanner, now try to connect to unreachable scanner
    ASSERT_THROW(phoxi_interface.connectCamera(incorrect_camera_ID), PhoXiScannerNotFound);
    ASSERT_TRUE(phoxi_interface.isConnected());     // I am still connected to previous scanner
    ASSERT_NO_THROW(phoxi_interface.isOk());

    //TODO exception PhoXiControNotRunning
}

TEST_F (PhoXiInterfaceTestConnection, connectWithAcquisition) {
    PhoXiInterface phoxi_interface = PhoXiInterface();

    pho::api::PhoXiTriggerMode triggerMode = pho::api::PhoXiTriggerMode::Software;

    //  connect to scanner and start acquisition
    ASSERT_NO_THROW(phoxi_interface.connectCamera(camera_ID, triggerMode, true));
    ASSERT_TRUE(phoxi_interface.isAcquiring());
    ASSERT_TRUE(phoxi_interface.isConnected());
    ASSERT_NO_THROW(phoxi_interface.isOk());
    phoxi_interface.disconnectCamera();

    phoxi_interface = PhoXiInterface();
    // connect to scanner without acquisition
    ASSERT_NO_THROW(phoxi_interface.connectCamera(camera_ID, triggerMode, false));
    ASSERT_FALSE(phoxi_interface.isAcquiring());
    ASSERT_TRUE(phoxi_interface.isConnected());
    ASSERT_NO_THROW(phoxi_interface.isOk());
    phoxi_interface.disconnectCamera();
}

TEST_F (PhoXiInterfaceTestConnection, disconnect) {
    PhoXiInterface phoxi_interface = PhoXiInterface();

    // connect and disconnect camera
    phoxi_interface.connectCamera(camera_ID);
    ASSERT_NO_THROW(phoxi_interface.isOk());
    ASSERT_NO_THROW(phoxi_interface.disconnectCamera());
    ASSERT_THROW(phoxi_interface.isOk(), PhoXiScannerNotConnected);

    // try disconnect camera when I am disconnected
    ASSERT_NO_THROW(phoxi_interface.disconnectCamera());
    ASSERT_THROW(phoxi_interface.isOk(), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, isOK) {

    // I am connected
    ASSERT_NO_THROW(phoxi_interface.isOk());

    phoxi_interface.disconnectCamera();

    ASSERT_THROW(phoxi_interface.isOk(), PhoXiScannerNotConnected);

    // TODO call isOK when PhoXi is off
}

TEST_F (PhoXiInterfaceTest, acquisition) {
    // is acquiring
    phoxi_interface.startAcquisition();
    ASSERT_TRUE(phoxi_interface.isAcquiring());

    // is not acquiring
    phoxi_interface.stopAcquisition();
    ASSERT_FALSE(phoxi_interface.isAcquiring());
    // when it is not acquiring, it is not able to trig
    ASSERT_EQ(phoxi_interface.triggerImage(), -1);


    //TODO try it without connected camera
}

TEST_F (PhoXiInterfaceTest, setTriggerMode) {
    pho::api::PhoXiTriggerMode triggerMode;

    // freerun
    triggerMode = pho::api::PhoXiTriggerMode::Freerun;
    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, true));
    EXPECT_TRUE(phoxi_interface.isAcquiring());

    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, false));
    EXPECT_FALSE(phoxi_interface.isAcquiring());
    EXPECT_EQ(-1, phoxi_interface.triggerImage());

    // software
    triggerMode = pho::api::PhoXiTriggerMode::Software;
    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, true));
    EXPECT_TRUE(phoxi_interface.isAcquiring());

    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, false));
    EXPECT_FALSE(phoxi_interface.isAcquiring());
    EXPECT_EQ(-1, phoxi_interface.triggerImage());

    // hardware
    triggerMode = pho::api::PhoXiTriggerMode::Hardware;
    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, true));
    EXPECT_TRUE(phoxi_interface.isAcquiring());

    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, false));
    EXPECT_FALSE(phoxi_interface.isAcquiring());
    EXPECT_EQ(-1, phoxi_interface.triggerImage());

    // no value
    triggerMode = pho::api::PhoXiTriggerMode::NoValue;
    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, true));
    EXPECT_TRUE(phoxi_interface.isAcquiring());

    ASSERT_NO_THROW(phoxi_interface.setTriggerMode(triggerMode, false));
    EXPECT_FALSE(phoxi_interface.isAcquiring());
    EXPECT_EQ(-1, phoxi_interface.triggerImage());

    // incorrect parameters
    ASSERT_ANY_THROW(phoxi_interface.setTriggerMode(-1, true));
    EXPECT_EQ(-1, phoxi_interface.triggerImage());

    ASSERT_ANY_THROW(phoxi_interface.setTriggerMode(-1, false));
    EXPECT_EQ(-1, phoxi_interface.triggerImage());

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.setTriggerMode(0, false), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, setResolution) {

    ASSERT_NO_THROW(phoxi_interface.setHighResolution());
    ASSERT_NO_THROW(phoxi_interface.setLowResolution());

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.setLowResolution(), PhoXiScannerNotConnected);
    ASSERT_THROW(phoxi_interface.setHighResolution(), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, triggerImage) {
    int imageCount;
    usleep(1000*1000*1);    // wait while phoxi trig first scan after connection

    pho::api::PhoXiTriggerMode triggerMode;

    // freerun
    triggerMode = pho::api::PhoXiTriggerMode::Freerun;
    phoxi_interface.setTriggerMode(triggerMode, true);
    imageCount = phoxi_interface.triggerImage();
    ASSERT_EQ(imageCount, 1);

    // software
    triggerMode = pho::api::PhoXiTriggerMode::Software;
    phoxi_interface.setTriggerMode(triggerMode, true);
    ASSERT_EQ(++imageCount, phoxi_interface.triggerImage());

    // hardware
    triggerMode = pho::api::PhoXiTriggerMode::Hardware;
    phoxi_interface.setTriggerMode(triggerMode, true);
    ASSERT_EQ(++imageCount, phoxi_interface.triggerImage());

    // no value
    triggerMode = pho::api::PhoXiTriggerMode::NoValue;
    phoxi_interface.setTriggerMode(triggerMode, true);
    ASSERT_EQ(++imageCount, phoxi_interface.triggerImage());

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.triggerImage(), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, getHardwareIdentification) {
    // function returns camera ID
    ASSERT_EQ(camera_ID, phoxi_interface.getHardwareIdentification());

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.getHardwareIdentification(), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, setTransformation) {
    // set transformation
    pho::api::PhoXiCoordinateTransformation transformation;
    transformation.Translation.x = 1;
    transformation.Translation.y = -5;
    transformation.Translation.z = 7;

    transformation.Rotation[0][0] = 1;
    transformation.Rotation[0][1] = 2;
    transformation.Rotation[0][2] = 4;

    transformation.Rotation[1][0] = -5;
    transformation.Rotation[1][1] = -1;
    transformation.Rotation[1][2] = 0;

    transformation.Rotation[2][0] = 1;
    transformation.Rotation[2][1] = 1;
    transformation.Rotation[2][2] = 0;

    // CustomSpace
    ASSERT_NO_THROW(phoxi_interface.setTransformation(transformation,
                                                      pho::api::PhoXiCoordinateSpace::CustomSpace,
                                                      true, true));
    ASSERT_NO_THROW(phoxi_interface.triggerImage());

    // RobotSpace
    ASSERT_NO_THROW(phoxi_interface.setTransformation(transformation,
                                                      pho::api::PhoXiCoordinateSpace::RobotSpace,
                                                      true, true));
    ASSERT_NO_THROW(phoxi_interface.triggerImage());


    // other like Custom and Robot space
    EXPECT_THROW(phoxi_interface.setTransformation(transformation,
                                                   pho::api::PhoXiCoordinateSpace::MarkerSpace,
                                                   true, true), CoordinationSpaceNotSupported);

    // try it without connected camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.setTransformation(transformation,
                                                   pho::api::PhoXiCoordinateSpace::CustomSpace,
                                                   true, true), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, setCoordinateSpace) {
    // try to set some space and next trig
    EXPECT_NO_THROW(phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::CameraSpace));
    EXPECT_NE(-1, phoxi_interface.triggerImage());

    EXPECT_NO_THROW(phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::CustomSpace));
    EXPECT_NE(-1, phoxi_interface.triggerImage());

    EXPECT_NO_THROW(phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::MarkerSpace));
    EXPECT_NE(-1, phoxi_interface.triggerImage());

    EXPECT_NO_THROW(phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::MountingSpace));
    EXPECT_NE(-1, phoxi_interface.triggerImage());

    EXPECT_NO_THROW(phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::NoValue));
    EXPECT_NE(-1, phoxi_interface.triggerImage());

    EXPECT_NO_THROW(phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::RobotSpace));
    EXPECT_NE(-1, phoxi_interface.triggerImage());

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::RobotSpace),
                 PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, getPFrame) {
    int bad_frameID = 1123;
    pho::api::PFrame frame;

    // trigger image and get PFrame
    int frameID = phoxi_interface.triggerImage();
    ASSERT_NE(nullptr, phoxi_interface.getPFrame(frameID));

    // get PFrame with triggerImage, triggerImage is called inside getPFrame when parameter is -1
    ASSERT_NE(nullptr, phoxi_interface.getPFrame(-1));

    // try getPFrame with bad ID, it have to return nullptr
    ASSERT_EQ(nullptr, phoxi_interface.getPFrame(bad_frameID));

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.getPFrame(-1), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, getPointCloudFromFrame) {
    ASSERT_THROW(phoxi_interface.getPointCloudFromFrame(nullptr), CorruptedFrame);

    pho::api::PFrame frame;
    frame = phoxi_interface.getPFrame(-1);
    ASSERT_NE(nullptr, phoxi_interface.getPointCloudFromFrame(frame));

    // try it without connection to camera
    frame = phoxi_interface.getPFrame(-1);
    phoxi_interface.disconnectCamera();
    // getPointCloudFromFrame only translate variable, it not need connection
    ASSERT_NE(nullptr, phoxi_interface.getPointCloudFromFrame(frame));
}

TEST_F (PhoXiInterfaceTest, getPointCloud) {
    ASSERT_NO_THROW(phoxi_interface.getPointCloud());

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.getPointCloud(), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, getSupportedCapturingModes) {
    vector<pho::api::PhoXiCapturingMode> modes;
    modes = phoxi_interface.getSupportedCapturingModes();

    for(auto mode: modes){
        ASSERT_NE(0, mode.Resolution.Width);
        ASSERT_NE(0, mode.Resolution.Height);
    }

    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_THROW(phoxi_interface.getSupportedCapturingModes(), PhoXiScannerNotConnected);
}

TEST_F (PhoXiInterfaceTest, cameraList) {
    // try it when I am connected to a camera
    ASSERT_NO_THROW(phoxi_interface.cameraList());

    cout << "Achievable cameras, you can check them, it must be equal to reality:"<<endl;
    for (auto camera : phoxi_interface.cameraList()) {
        cout << camera << ", ";
    }
    cout<<endl;


    // try it without connection to camera
    phoxi_interface.disconnectCamera();
    ASSERT_NO_THROW(phoxi_interface.cameraList());

    cout << "Achievable cameras, you can check them, it must be equal to reality:"<<endl;
    for (auto camera : phoxi_interface.cameraList()) {
        cout << camera << ", ";
    }
}