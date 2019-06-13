#include <iostream>

#include "../utilities/range.hpp"
#include "../utilities/stop_watch.hh"
#include "../utilities/pico_sense.h"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

int main() {
    pico_init_fusion();
    std::cout << "started" << std::endl;
    
    PsFrame         depth_frame, rgb_frame;
    stop_watch      _clock;
    Eigen::VectorXf plane = Eigen::VectorXf::Zero(4, 1);
    pcl::RandomSampleConsensus<pcl::PointXYZRGB>
                    ransac(nullptr);
    ransac.setDistanceThreshold(10);
    ransac.setProbability(0.5);
    ransac.setMaxIterations(16);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::CloudViewer        viewer("view");
    
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
        
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
            model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud));
        ransac.setSampleConsensusModel(model_p);
        
        std::cout << "----------------------------" << std::endl;
        _clock.reset();
        ransac.computeModel();
        std::cout << "fps:   " << 1.0 / _clock.seconds() << std::endl;
        
        std::vector<int> inliers{};
        ransac.getInliers(inliers);
        
        ransac.getModelCoefficients(plane);
        
        std::cout << "rate:  " << static_cast<float>(inliers.size()) / cloud->size() << std::endl
                  << "plane: ";
        float t    = 0;
        auto  i    = 0;
        auto  data = plane.data();
        while (i < 3) {
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
        
        model_p->selectWithinDistance(plane, 20, inliers);
        
        for (auto j : inliers) {
            cloud->points[j].r = 0;
            cloud->points[j].g = 128;
            cloud->points[j].b = 0;
        }
        
        viewer.showCloud(cloud);
    }
}
