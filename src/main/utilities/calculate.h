//
// Created by user on 2019/6/26.
//

#ifndef PCL_TEST_CALCULATE_H
#define PCL_TEST_CALCULATE_H

#include <pcl/common/common_headers.h>

#include <sstream>

struct sac_result_t {
    double          calculation, total, rate;
    Eigen::VectorXf model = Eigen::VectorXf::Zero(4, 1);
    
    [[nodiscard]] std::string to_string(const std::string &title) const {
        std::stringstream builder;
        builder << title << ": calculate / total = "
                << calculation << "s / " << total << "s = "
                << calculation / total * 100 << "% | "
                << "time not for calcuation: " << total - calculation << "s";
        return builder.str();
    }
};

sac_result_t ransac_cuda(
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    size_t max_times,
    float distance,
    float probability
);

sac_result_t ransac_cpu(
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    size_t max_times,
    float distance,
    float probability
);

#endif //PCL_TEST_CALCULATE_H
