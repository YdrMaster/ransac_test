#include <iostream>
#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

#include "../../extern-lib/PicoZenseSDK_Windows_20190316_V2.3.9.2_DCAM710/Include/PicoZense_api.h"

int main() {
    PsInitialize();
    
    int count;
    PsGetDeviceCount(&count);
    if (count < 1) throw std::runtime_error("no camera");
    
    PsOpenDevice(0);
    
    
    PsCloseDevice(0);
    PsShutdown();
    
    return 0;
}
