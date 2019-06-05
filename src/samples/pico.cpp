#include <iostream>
#include <chrono>

#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

#include "../../extern-lib/PicoZenseSDK_Windows_20190316_V2.3.9.2_DCAM710/Include/PicoZense_api.h"
#include "range.hpp"

int main() {
    PsInitialize();
    
    int count;
    PsGetDeviceCount(&count);
    if (count < 1) throw std::runtime_error("no camera");
    
    PsOpenDevice(0);
    
    PsFrame                           depth_frame;
    std::vector<plane_t<3>::_point_t> points{};
    
    while (true) {
        PsReadNextFrame(0);
        PsGetFrame(0, PsDepthFrame, &depth_frame);
        
        if (!depth_frame.pFrameData) continue;
        
        const auto x0 = depth_frame.width / 2,
                   y0 = depth_frame.height / 2;
        const auto n  = depth_frame.height * depth_frame.width;
        
        auto depth_data  = (PsDepthPixel *) depth_frame.pFrameData;
        
        for (auto i : range_t<size_t>(0, n - 1)) {
            if (depth_data[i] > 0)
                points.emplace_back(point_t<3>{
                    static_cast<float>(i % depth_frame.width - x0),
                    static_cast<float>(y0 - i / depth_frame.width),
                    static_cast<float>(-depth_data[i])
                });
        }
        auto      time   = std::chrono::steady_clock::now();
        auto      result = ransac<plane_t<3>>(points, 0.05f, 0.5, 10000);
        std::cout << "----------------------------" << std::endl
                  << static_cast<float >( result.inliers.size()) / points.size() << std::endl
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time).count()
                  << std::endl;
    }
    
    PsCloseDevice(0);
    PsShutdown();
    
    return 0;
}
