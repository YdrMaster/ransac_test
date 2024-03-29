﻿//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_RANSAC_CU
#define RANSAC_RANSAC_CU


#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <array>
#include <algorithm>
#include "models/model_t.hpp"
#include "random_engine.hpp"

template<class _model_t>
struct ransac_result_t {
    _model_t            model{};
    std::vector<size_t> inliers{};
    float               rate = 0;
};

template<class _model_t>
__global__ void map(typename _model_t::super_t::_point_t *ptr_i,
                    bool *ptr_o, 
                    _model_t model,
                    float threshold) {
    const int i = blockIdx.x*blockDim.x + threadIdx.x;
    ptr_o[i] = std::abs(model(ptr_i[i])) < threshold;
}

template<class _model_t>
ransac_result_t<_model_t>
ransac(const std::vector<typename _model_t::super_t::_point_t> &data,
       float threshold,
       float success_rate = 0.99f,
       size_t max_times = std::numeric_limits<size_t>::max(),
       const _model_t &guess = _model_t{}
) {
    using tp = typename _model_t::super_t::_point_t;
    using ti = decltype(data.size());
    
    const auto size         = data.size(),
               success_size = static_cast<ti>(success_rate * size);
    
    if (size < 2 * _model_t::super_t::size_to_make)
        throw std::logic_error("samples too little");
    
    // 随机数引擎
    random_engine<ti>
         random(0, size - 1);
    // 模型初始化器
    std::array<tp, _model_t::super_t::size_to_make>
         initialize_list{};
    // 缓存
    tp   *point_buffer = nullptr;
    bool *check_buffer = nullptr;
    
    cudaMallocManaged(&point_buffer, size * sizeof(tp));
    cudaMallocManaged(&check_buffer, size * sizeof(bool));

    std::copy(data.begin(), data.end(), point_buffer);
		
    ti       count      = 0;
    _model_t best_model = guess,
             model{};
    
    if (best_model.is_valid()) {
        map<<<(size - 1) / 32 + 1, 32>>>(point_buffer, check_buffer, best_model, threshold);
	    cudaDeviceSynchronize();
    
        count = std::count(check_buffer, check_buffer + size, true);
    }
    
    for (; max_times > 0 && count < success_size; --max_times) {
        for (ti i = 0; i < initialize_list.size(); ++i)
            initialize_list[i] = data[random()];
        
        model.make(initialize_list);
        if (!model.is_valid() || (best_model.is_valid() && model == best_model))
            continue;
        
        map<<<(size - 1) / 32 + 1, 32>>>(point_buffer, check_buffer, model, threshold);
	    cudaDeviceSynchronize();
        
        ti temp = std::count(check_buffer, check_buffer + size, true);
        if (temp > count) {
            count      = temp;
            best_model = model;
        }
    }
    
    // 局内点
    std::vector<ti> inliers(count);
    
    auto    ptr = inliers.begin();
    for (ti i   = 0; i < size; ++i)
        if (check_buffer[i]) *ptr++ = i;

    cudaFree(point_buffer);
    cudaFree(check_buffer);
    
    auto rate = static_cast<float>(inliers.size()) / size;
    return {best_model, std::move(inliers), rate};
}


#endif // RANSAC_RANSAC_CU
