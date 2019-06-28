//
// Created by user on 2019/6/28.
//

#include <iostream>

#include "../main/utilities/range.hpp"
#include "../main/utilities/stop_watch.hh"
#include "../main/utilities/pico_sense.h"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/sample_consensus/sac_model_plane.h>

#include "../main/random_engine.hpp"

#include "../main/utilities/calculate.h"

int main() {
    using point_t  = pcl::PointXYZRGB;
    using cloud_t  = pcl::PointCloud<point_t>;
    using viewer_t = pcl::visualization::CloudViewer;
    using vector_t = Eigen::VectorXf;
    using plane_t  = pcl::SampleConsensusModelPlane<point_t>;
    
    pico_init_fusion();
    std::cout << "started" << std::endl;
    
    PsFrame    depth_frame, rgb_frame;
    stop_watch _clock;
    
    cloud_t::Ptr cloud(new cloud_t);
    viewer_t     viewer("view");
    
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
        
        cloud->clear();
        for (auto i : range_t<size_t>(0, n - 1)) {
            const auto z = static_cast<float>(-depth_data[i]);
            if (z >= 0) continue;
            
            pcl::PointXYZRGB point;
            point.x = static_cast<float>(i % depth_frame.width) - x0;
            point.y = y0 - static_cast<float>(i / depth_frame.width);
            point.z = z;
            point.r = rgb_data[i].r;
            point.g = rgb_data[i].g;
            point.b = rgb_data[i].b;
            cloud->push_back(point);
        }
        
        auto result = ransac_cuda(cloud, 1024, 10, 0.75);
        
        auto plane = result.model;
        
        std::cout << "fps:   " << 1 / result.calculation << std::endl
                  << "rate:  " << result.rate << std::endl
                  << "plane: ";
        float t    = 0;
        auto  i    = 0;
        auto  data = plane.data();
        while (i < plane.rows() - 1) {
            if (data[i] != 0) {
                if (t != 0)
                    std::cout << data[i] / t << ' ';
                else
                    t = data[i];
                std::cout << 'x' << i << " + ";
            }
            ++i;
        }
        std::cout << data[i] << " == 0" << std::endl;
        
        //        std::vector<int> inliers{};
        //        plane_t::Ptr(new plane_t(cloud))->selectWithinDistance(plane, 20, inliers);
        //
        //        for (auto j : inliers) {
        //            cloud->points[j].r = 0;
        //            cloud->points[j].g = 128;
        //            cloud->points[j].b = 0;
        //        }
        
        viewer.showCloud(cloud);
    }
    
    return 0;
}
