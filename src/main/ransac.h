﻿//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H


#include "models/model_t.hpp"
#include "random_engine.hpp"

template<class _model_t>
struct ransac_result {
    _model_t            model;
    std::vector<size_t> inliers;
};

template<class _model_t>
ransac_result<_model_t> ransac(
    const std::vector<typename _model_t::_point_t> &data,
    float threshold,
    unsigned int max_times
) {
    using tp = typename _model_t::_point_t;
    
    if (data.size() < 2 * _model_t::size_to_make)
        throw std::logic_error("samples too little");
    
    // 随机数引擎
    random_engine<decltype(data.size())>
        random(0, data.size() - 1);
    // 模型初始化器
    std::array<tp, _model_t::size_to_make>
        initialize_list{};
    // 缓存
    std::vector<bool>
        check_buffer(data.size());
    // 局内点
    std::vector<size_t>
        inliers{};
    
    _model_t best_model{}, model{};
    
    for (size_t i = 0; i < max_times; ++i) {
        for (size_t j = 0; j < initialize_list.size(); ++j)
            initialize_list[j] = data[random()];
        
        model.make(initialize_list);
        if (!model.is_valid() || (best_model.is_valid() && model == best_model))
            continue;
        
        std::transform(data.begin(), data.end(),
                       check_buffer.begin(),
                       [&](const tp &point) {
                           return std::abs(model(point)) < threshold;
                       });
        
        size_t count = std::count(check_buffer.begin(), check_buffer.end(), true);
        if (count <= inliers.size()) continue;
        
        best_model = model;
        inliers.resize(count);
        
        auto        ptr = inliers.begin();
        for (size_t i   = 0; i < check_buffer.size(); ++i)
            if (check_buffer[i]) *ptr++ = i;
    }
    
    return {best_model, inliers};
}


#endif // RANSAC_RANSAC_H