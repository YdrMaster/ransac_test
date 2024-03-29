﻿//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_MODEL_T_HPP
#define RANSAC_MODEL_T_HPP


#include "point_t.hpp"

constexpr static auto float_equal = 1E-6f;

/**
 * 模型
 *
 * @tparam _point_dim     点维度
 * @tparam _size_to_make  确定模型所需最小点数
 */
template<
    size_t _point_dim,
    size_t _size_to_make
> struct model_t {
    using _point_t = point_t<_point_dim>;
    
    constexpr static auto
        point_dim    = _point_dim,
        size_to_make = _size_to_make;
    
    // =====================
    // void make(const std::array<_point_t, size_to_make> &)
    //
    // float operator()(const _point_t &) const
    //
    // bool is_valid() const
    //
    // bool operator==(const model_t &) const
    //
    // bool operator!=(const model_t &) const
    // =====================
};


#endif //RANSAC_MODEL_T_HPP
