//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H


#include <vector>
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
    std::vector<bool> check_buffer(size);
    
    ti       count      = 0;
    _model_t best_model = guess,
             model{};
    
    if (best_model.is_valid()) {
        std::transform(data.begin(), data.end(), check_buffer.begin(),
                       [=](const tp &point) { return std::abs(best_model(point)) < threshold; });
    
        count = std::count(check_buffer.begin(), check_buffer.end(), true);
    }
    
    for (; max_times > 0 && count < success_size; --max_times) {
        for (ti i = 0; i < initialize_list.size(); ++i)
            initialize_list[i] = data[random()];
        
        model.make(initialize_list);
        if (!model.is_valid() || (best_model.is_valid() && model == best_model))
            continue;
    
        std::transform(data.begin(), data.end(), check_buffer.begin(),
                       [=](const tp &point) { return std::abs(model(point)) < threshold; });
    
        ti temp = std::count(check_buffer.begin(), check_buffer.end(), true);
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
    
    auto rate = static_cast<float>(inliers.size()) / size;
    return {best_model, std::move(inliers), rate};
}


#endif // RANSAC_RANSAC_H
