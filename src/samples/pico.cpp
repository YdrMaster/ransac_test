#include <iostream>
#include <chrono>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

#include <PicoZense_api.h>

#include "range.hpp"

constexpr auto display_range = 20; // mm

int main() {
    PsInitialize();
    
    int count;
    PsGetDeviceCount(&count);
    if (count < 1) throw std::runtime_error("no camera");
    
    PsOpenDevice(0);
    PsSetDepthRange(0, PsNearRange);
    PsSetDataMode(0, PsDepthAndRGB_30);
    PsSetMapperEnabledDepthToRGB(0, true);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::CloudViewer        viewer("view");
    
    plane_t<3> plane{};
    
    PsFrame depth_frame, rgb_frame;
    
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
            float x = static_cast<float>(i % depth_frame.width) - x0,
                  y = y0 - static_cast<float>(i / depth_frame.width),
                  z = static_cast<float>(-depth_data[i]);
    
            if (depth_data[i] > 0) {
                points.emplace_back(point_t<3>{x, y, z});
                rgbs.push_back(rgb_data[i]);
            }
        }
    
        auto time = std::chrono::steady_clock::now();
    
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
                  << "fps:   " << 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time).count()
                  << std::endl;
    
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
