//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_PLANE_T_HPP
#define RANSAC_PLANE_T_HPP


#include "model_t.hpp"

template<size_t _dim>
struct plane_t final : public model_t<_dim, _dim> {
private:
    static_assert(_dim >= 2, "plane dimension is at least 2");
    
    using super_t   = model_t<_dim, _dim>;
    using __point_t = typename super_t::_point_t;
    
    float _norm = -1;

public:
    __point_t normal{}; // 法向量
    float     b = 0;    // 截距
    
    void make(const std::array<__point_t, super_t::size_to_make> &points) final {
        _norm = normal.norm();
    }
    
    float operator()(const __point_t &point) const final {
        return std::fabsf(normal * point + parameters.back()) / _norm;
    }
    
    bool is_valid() const final {
        return _norm > float_equal;
    }
    
    bool operator==(const plane_t &others) const {
        return (normal - others.normal).norm(1) > float_equal
               || std::fabsf(b - others.b) > float_equal;
    }
    
    bool operator!=(const plane_t &others) const {
        return !operator==(others);
    }
};


#endif // RANSAC_PLANE_T_HPP
