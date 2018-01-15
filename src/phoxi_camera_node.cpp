/*********************************************************************************************//**
* @file phoxi_camera_node.cpp
*
* Copyright (c)
* Photoneo s.r.o
* November 2016
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/

#include <phoxi_camera/RosInterface.h>
/*void init_config(pho::api::PPhoXi &Scanner) {
    std::cout << "cinit" << std::endl;
//    ros::param::set("~size_height", Scanner->Resolution->Height);
//    ros::param::set("~size_width", Scanner->Resolution->Width);
    int idx = 0;
    std::vector<pho::api::PhoXiCapturingMode> capturingModes = EvaluationScanner->SupportedCapturingModes;
    for (int i = 0; i < capturingModes.size(); ++i) {
        if(capturingModes[i] == Scanner->CapturingMode){
            idx=i;
            break;
        }
    }
    std::cout << "Capture mode: " << idx << std::endl;
    if (capturingModes.size() > idx) {
        EvaluationScanner->CapturingMode = capturingModes[idx];
    }
    ros::param::set("~vertical_resolution", idx+1);
    ros::param::set("~horizontal_resolution", idx+1);
    ros::param::set("~scan_multiplier", Scanner->CapturingSettings->ScanMultiplier);
    ros::param::set("~shutter_multiplier", Scanner->CapturingSettings->ShutterMultiplier);
//    ros::param::set("~acquisition_time", Scanner->AcquisitionTime);
    ros::param::set("~trigger_mode", (pho::api::PhoXiTriggerMode::Value)(pho::api::PhoXiTriggerMode)Scanner->TriggerMode);
    ros::param::set("~timeout", (int)(pho::api::PhoXiTimeout)Scanner->Timeout);
    ros::param::set("~confidence", Scanner->ProcessingSettings->Confidence);
    ros::param::set("~send_point_cloud", Scanner->OutputSettings->SendPointCloud);
    ros::param::set("~send_normal_map", Scanner->OutputSettings->SendNormalMap);
    ros::param::set("~send_confidence_map", Scanner->OutputSettings->SendConfidenceMap);
    ros::param::set("~send_texture", Scanner->OutputSettings->SendTexture);
}

void callback(pho::api::PPhoXi &Scanner, phoxi_camera::phoxi_cameraConfig &config, uint32_t level) {
    if (EvaluationScanner == 0){
        return;
    }
//    if (level & (1 << 1)) {
//        Scanner->Resolution->Height = config.size_height;
//    }
//    if (level & (1 << 2)) {
//        Scanner->Resolution->Width = config.size_width;
//    }
    if (level & (1 << 3)) {
        std::vector<pho::api::PhoXiCapturingMode> capturingModes = EvaluationScanner->SupportedCapturingModes;
        if (capturingModes.size() >= config.vertical_resolution) {
            EvaluationScanner->CapturingMode = capturingModes[config.vertical_resolution - 1];
            config.horizontal_resolution = config.vertical_resolution;
        }
//        Scanner->CapturingMode->Resolution.Height = config.capturing_size_width;
    }
    if (level & (1 << 4)) {
//        Scanner->CapturingMode->Resolution.Height = config.horizontal_resolution;
        std::vector<pho::api::PhoXiCapturingMode> capturingModes = EvaluationScanner->SupportedCapturingModes;
        if (capturingModes.size() >= config.horizontal_resolution) {
            EvaluationScanner->CapturingMode = capturingModes[config.horizontal_resolution - 1];
            config.vertical_resolution = config.horizontal_resolution;
        }
    }
    if (level & (1 << 5)) {
        Scanner->CapturingSettings->ScanMultiplier = config.scan_multiplier;
    }
//    if (level & (1 << 6)) {
//        Scanner->AcquisitionTime = config.acquisition_time;
//    }
    if (level & (1 << 6)) {
        Scanner->CapturingSettings->ShutterMultiplier = config.shutter_multiplier;
    }
    if (level & (1 << 7)) {
        Scanner->TriggerMode = config.trigger_mode;
    }
    if (level & (1 << 8)) {
        Scanner->Timeout = config.timeout;
    }
    if (level & (1 << 9)) {
        Scanner->ProcessingSettings->Confidence = config.confidence;
    }
    if (level & (1 << 10)) {
        Scanner->OutputSettings->SendPointCloud = config.send_point_cloud;
    }
    if (level & (1 << 11)) {
        Scanner->OutputSettings->SendNormalMap = config.send_normal_map;
    }
    if (level & (1 << 12)) {
        Scanner->OutputSettings->SendConfidenceMap = config.send_confidence_map;
    }
    if (level & (1 << 13)) {
        Scanner->OutputSettings->SendTexture = config.send_texture;
    }
}*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "phoxi_camera");

    /*dynamic_reconfigure::Server <phoxi_camera::phoxi_cameraConfig> server;
    dynamic_reconfigure::Server<phoxi_camera::phoxi_cameraConfig>::CallbackType f;

    f = boost::bind(&callback, boost::ref(EvaluationScanner), _1, _2);
    server.setCallback(f);*/
    RosInterface interface;

    ros::spin();
    return 0;
}