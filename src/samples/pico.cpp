#include <iostream>
#include <chrono>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

#include <PicoZense_api.h>

#include "range.hpp"

int main() {
    PsInitialize();
    
    int count;
    PsGetDeviceCount(&count);
    if (count < 1) throw std::runtime_error("no camera");
    
    PsOpenDevice(0);
    PsSetDepthRange(0, PsNearRange);
    PsSetDataMode(0, PsDepth_60);
    
    PsFrame depth_frame;
    
    pcl::visualization::CloudViewer viewer("view");
    
    plane_t<3> plane{};
    
    while (!viewer.wasStopped()) {
        PsReadNextFrame(0);
        PsGetFrame(0, PsDepthFrame, &depth_frame);
        
        if (!depth_frame.pFrameData) continue;
        
        const auto x0 = depth_frame.width / 2,
                   y0 = depth_frame.height / 2;
        const auto n  = depth_frame.height * depth_frame.width;
    
        auto depth_data = (PsDepthPixel *) depth_frame.pFrameData;
    
        std::vector<plane_t<3>::_point_t> points{};
        
        for (auto i : range_t<size_t>(0, n - 1)) {
            if (depth_data[i] > 0)
                points.emplace_back(point_t<3>{
                    static_cast<float>(i % depth_frame.width) - x0,
                    y0 - static_cast<float>(i / depth_frame.width),
                    static_cast<float>(-depth_data[i])
                });
        }
    
        auto time   = std::chrono::steady_clock::now();
        auto result = ransac<plane_t<3>>(points, 10, 0.5, 32, plane);
        std::cout << "----------------------------" << std::endl
                  << "rate:   " << result.rate << std::endl
                  << "normal: " << result.model.normal << std::endl
                  << "time:   " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time).count()
                  << std::endl;
    
        plane = result.model;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(result.inliers.size());
        for (size_t i = 0; i < result.inliers.size(); ++i) {
            pcl::PointXYZ point;
            auto          t = result.inliers[i];
            point.x = points[t].x();
            point.y = points[t].y();
            point.z = points[t].z();
            cloud->points[i] = point;
        }
        viewer.showCloud(cloud);
    }
    
    PsCloseDevice(0);
    PsShutdown();
    
    return 0;
}
