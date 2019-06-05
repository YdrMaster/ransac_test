//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_PLANE_T_HPP
#define RANSAC_PLANE_T_HPP


#include "model_t.hpp"

template<size_t _dim>
struct plane_t final : public model_t<_dim, _dim> {};

template<>
struct plane_t<3> final : public model_t<3, 3> {
    _point_t normal{NAN, NAN, NAN}, // 单位法向量
             _point{};              // 平面上任一点
    
    void make(const std::array<_point_t, size_to_make> &points) final {
        const auto t0 = points[2] - points[0],
                   t1 = points[1] - points[0];
        _point = points[0];
        normal = {+t0.y() * t1.z() - t1.y() * t0.z(),
                  -t0.x() * t1.z() + t1.x() * t0.z(),
                  +t0.x() * t1.y() - t1.x() * t0.y()};
        auto norm = normal.norm();
        if (norm < float_equal)
            normal = {NAN, NAN, NAN};
        else
            normal /= norm;
    }
    
    float operator()(const _point_t &point) const final {
        return std::fabsf(normal * (point - _point));
    }
    
    bool is_valid() const final { return normal.is_valid(); }
    
    bool operator==(const plane_t &others) const {
        return (normal - others.normal).norm(1) < float_equal
               && (_point - others._point).norm(1) < float_equal;
    }
    
    bool operator!=(const plane_t &others) const {
        return !operator==(others);
    }
};


#endif // RANSAC_PLANE_T_HPP
