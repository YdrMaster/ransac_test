//
// Created by user on 6/12/19.
//

#ifndef RANSAC_PICO_SENSE_H
#define RANSAC_PICO_SENSE_H


#include <PicoZense_api.h>
#include <stdexcept>

void pico_init_depth() {
    PsInitialize();
    
    int count;
    PsGetDeviceCount(&count);
    if (count < 1) throw std::runtime_error("no camera");
    
    PsOpenDevice(0);
    PsSetDepthRange(0, PsNearRange);
    PsSetDataMode(0, PsDepth_60);
}

void pico_init_fusion() {
    PsInitialize();
    
    int count;
    PsGetDeviceCount(&count);
    if (count < 1) throw std::runtime_error("no camera");
    
    PsOpenDevice(0);
    PsSetDepthRange(0, PsNearRange);
    PsSetDataMode(0, PsDepthAndRGB_30);
    PsSetMapperEnabledDepthToRGB(0, true);
}

#endif //RANSAC_PICO_SENSE_H
