//
// Created by user on 2019/6/26.
//

#include "calculate.h"
#include "stop_watch.hh"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

sac_result_t ransac_cpu(
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    size_t max_times,
    float distance,
    float probability) {
    
    stop_watch clock;
    
    using sac_model_t = pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>;
    using sac_t       = pcl::RandomSampleConsensus<pcl::PointXYZRGB>;
    typename sac_model_t::Ptr sac_model(new sac_model_t(cloud));
    sac_t                     sac(sac_model, distance);
    sac.setMaxIterations(max_times);
    sac.setProbability(probability);
    
    auto calculate_time = clock.seconds();
    sac.computeModel();
    auto time = clock.seconds();
    
    sac_result_t result{
        time - calculate_time,
        time,
        static_cast<double>(sac.inliers_.size()) / cloud->size()
    };
    
    sac.getModelCoefficients(result.model);
    
    return result;
}
