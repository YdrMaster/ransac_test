#include "calculate.h"
#include "stop_watch.hh"

#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/sample_consensus/ransac.h>
#include <pcl/cuda/sample_consensus/sac_model_plane.h>

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
    
    auto data = toStorage<Host, Device>(host_cloud);
    using sac_model_t = SampleConsensusModelPlane<Device>;
    using sac_t = RandomSampleConsensus<Device>;
    typename sac_model_t::Ptr sac_model(new sac_model_t(data));
    sac_t                     sac(sac_model, distance);
    sac.setMaxIterations(max_times);
    sac.setProbability(probability);
    
    auto calculate_time = clock.seconds();
    auto success        = sac.computeModel();
    auto time           = clock.seconds();
    
    if (!success) return {};
    
    sac_result_t result{
        time - calculate_time,
        time
    };
    
    Device<float>::type device_vector(4);
    Host<float>::type   host_vector(4);
    sac.getModelCoefficients(device_vector);
    thrust::copy(device_vector.begin(), device_vector.end(), host_vector.begin());
    //    result.rate = static_cast<double>(sac_model->countWithinDistance(device_vector, distance)) / cloud->size();
    
//    for (int j = 0; j < 4; ++j) result.model[j] = host_vector[j];
    
    return result;
}
