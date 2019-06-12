#include <iostream>

#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

#include "../utilities/range.hpp"
#include "../utilities/stop_watch.hh"
#include "../utilities/pico_sense.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    pico_init_depth();
    std::cout << "started" << std::endl;
    
    plane_t<3> plane{};
    PsFrame    depth_frame;
    stop_watch _clock;
    
    while (true) {
        PsReadNextFrame(0);
        PsGetFrame(0, PsDepthFrame, &depth_frame);
    
        if (!depth_frame.pFrameData) continue;
        
        const auto x0 = depth_frame.width / 2,
                   y0 = depth_frame.height / 2;
        const auto n  = depth_frame.height * depth_frame.width;
    
        auto depth_data = (PsDepthPixel *) depth_frame.pFrameData;
    
        std::vector<point_t<3>> points{};
        
        for (auto i : range_t<size_t>(0, n - 1)) {
            const auto x = static_cast<float>(i % depth_frame.width) - x0,
                       y = y0 - static_cast<float>(i / depth_frame.width),
                       z = static_cast<float>(-depth_data[i]);
    
            if (z < 0) points.emplace_back<point_t<3>>({x, y, z});
        }
    
        _clock.reset();
        
        ransac_result_t<plane_t<3>> result{};
        try {
            result = ransac<plane_t<3>>(points, 10, 0.5, 16, plane);
        } catch (std::exception &e) {
            continue;
        }
    
        plane = result.model;
        
        std::cout << "----------------------------" << std::endl
                  << "rate:  " << result.rate << std::endl
                  << "plane: " << plane << std::endl
                  << "fps:   " << 1.0 / _clock.seconds() << std::endl;
    }
}

#pragma clang diagnostic pop
