#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

#include "../utilities/range.hpp"
#include "../utilities/stop_watch.hh"
#include "../utilities/pico_sense.h"

constexpr auto display_range = 20; // mm

int main() {
    pico_init_fusion();
    
    std::cout << "started" << std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::CloudViewer        viewer("view");
    
    plane_t<3> plane{};
    
    PsFrame depth_frame, rgb_frame;
    
    stop_watch _clock;
    
    while (!viewer.wasStopped()) {
        PsReadNextFrame(0);
        PsGetFrame(0, PsDepthFrame, &depth_frame);
        PsGetFrame(0, PsMappedRGBFrame, &rgb_frame);
    
        if (!depth_frame.pFrameData || !rgb_frame.pFrameData) continue;
        
        const auto x0 = depth_frame.width / 2,
                   y0 = depth_frame.height / 2;
        const auto n  = depth_frame.height * depth_frame.width;
    
        auto depth_data = (PsDepthPixel *) depth_frame.pFrameData;
        auto rgb_data   = (PsBGR888Pixel *) rgb_frame.pFrameData;
    
        std::vector<point_t<3>>    points{};
        std::vector<PsBGR888Pixel> rgbs{};
        
        for (auto i : range_t<size_t>(0, n - 1)) {
            const auto x = static_cast<float>(i % depth_frame.width) - x0,
                       y = y0 - static_cast<float>(i / depth_frame.width),
                       z = static_cast<float>(-depth_data[i]);
    
            if (z < 0) {
                points.emplace_back<point_t<3>>({x, y, z});
                rgbs.push_back(rgb_data[i]);
            }
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
        
        if (result.rate < 0.05) continue;
    
        cloud->clear();
        for (int i = 0; i < points.size(); ++i) {
            auto point = points[i];
            auto rgb   = rgbs[i];
    
            pcl::PointXYZRGB temp;
            temp.x = point.x();
            temp.y = point.y();
            temp.z = point.z();
            if (plane(point) < display_range) {
                temp.r = 0;
                temp.g = 128;
                temp.b = 0;
            } else {
                temp.r = std::min(255, 32 + rgb.r);
                temp.g = std::min(255, 32 + rgb.g);
                temp.b = std::min(255, 32 + rgb.b);
            }
    
            cloud->points.push_back(temp);
        }
        viewer.showCloud(cloud);
    }
    
    PsCloseDevice(0);
    
    PsShutdown();
    
    return 0;
}
