#include "calculate.h"
#include "stop_watch.hh"

#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/sample_consensus/multi_ransac.h>
#include <pcl/cuda/sample_consensus/sac_model_plane.h>

#include <cstdio>

sac_result_t ransac_cuda(
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    size_t max_times,
    float distance,
    float probability) {
    
    using namespace pcl::cuda;
    
    stop_watch clock;
    
    PointCloudAOS<Host> host_cloud;
    
    const auto size = cloud->size();
    host_cloud.points.resize(size);
    host_cloud.width  = cloud->width;
    host_cloud.height = cloud->height;
    for (size_t i = 0; i < size; ++i) {
        auto        p0 = cloud->points[i];
        PointXYZRGB p;
        p.x            = p0.x;
        p.y            = p0.y;
        p.z            = p0.z;
        host_cloud.points[i] = p;
    }
    
    using sac_model_t = SampleConsensusModelPlane<Device>;
    using sac_t = MultiRandomSampleConsensus<Device>;
    
    auto data = toStorage<Host, Device>(host_cloud);
    data->height = cloud->height;
    data->width  = cloud->width;
    
    typename sac_model_t::Ptr sac_model(new sac_model_t(data));
    
//    auto normals = computeFastPointNormals<Device>(data);
//    sac_model->setNormals(normals);
    
    sac_t sac(sac_model, distance);
    sac.setMinimumCoverage(0.9);
    sac.setMaximumBatches(1);
    sac.setIerationsPerBatch(2000);
    //    sac.setMaxIterations(max_times);
    //    sac.setProbability(probability);
    
    auto calculate_time = clock.seconds();
    auto success        = sac.computeModel(0);
    auto time           = clock.seconds();
    
    if (!success) return {-1, -1};
    
    sac_result_t result{time - calculate_time, time};
    
    auto temp = sac.getAllModelCoefficients();
    
    if (!temp.empty()) {
        result.model[0] = temp[0].x;
        result.model[1] = temp[0].y;
        result.model[2] = temp[0].z;
        result.model[3] = temp[0].w;
    }
    
    //    Device<float>::type device_vector(4);
    //    sac.getModelCoefficients(device_vector);
    //    Host<float>::type host_vector = device_vector;
    //    thrust::copy(device_vector.begin(), device_vector.end(), host_vector.begin());
    //    result.rate = static_cast<double>(sac_model->countWithinDistance(device_vector, distance)) / cloud->size();
    
    return result;
}
