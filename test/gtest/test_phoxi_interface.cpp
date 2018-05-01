//
// Created by controller on 4/18/18.
//

#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera/PhoXiException.h"
#include "test_phoxi_interface.h"
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string>

//doc
// https://github.com/google/googletest/blob/master/googletest/docs/AdvancedGuide.md

//examples
// https://github.com/google/googletest/blob/master/googletest/docs/Samples.md


using namespace std;
using namespace config;


class PhoXiInterfaceTest : public testing::Test {
public:
    static void SetUpTestCase(){
        //test constructor
        ASSERT_NO_THROW(PhoXiInterface());
    }
};

class DISABLED_PhoXiInterfaceTest : public testing::Test {
public:
    static void SetUpTestCase(){
    }
};

/*
 *
 */
TEST_F (DISABLED_PhoXiInterfaceTest, connect) {
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
    ASSERT_FALSE(phoxi_interface.isConnected());
    ASSERT_THROW(phoxi_interface.isOk(), PhoXiScannerNotConnected); // TODO?? I am still connected to previous scanner

    //TODO exception PhoXiControNotRunning
}

TEST_F (PhoXiInterfaceTest, connect_with_acquisition) {
    PhoXiInterface phoxi_interface = PhoXiInterface();

    int triggerMode = 1; //enum Value { Freerun = 0, Software = 1, Hardware = 2, NoValue = 3 }

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

TEST_F (DISABLED_PhoXiInterfaceTest, disconnect) {
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

TEST_F (DISABLED_PhoXiInterfaceTest, isOK) {
    PhoXiInterface phoxi_interface = PhoXiInterface();

    ASSERT_THROW(phoxi_interface.isOk(), PhoXiScannerNotConnected);

    phoxi_interface.connectCamera(camera_ID);

    ASSERT_NO_THROW(phoxi_interface.isOk());

    // TODO call isOK when PhoXi is off
}