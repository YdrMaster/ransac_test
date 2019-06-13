//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_PLANE_T_HPP
#define RANSAC_PLANE_T_HPP


#include <array>
#include <vector>

#include "macro.h"
#include "model_t.hpp"

template<size_t _dim>
struct _plane_t {
private:
    float square = -1;
    
    using super_t  = model_t<_dim, _dim>;

public:
    constexpr static auto dim = _dim;
    
    typename super_t::_point_t
          normal = {}; // 单位法向量
    float b      = 0;  // ...
    
    void make(const typename super_t::_point_t &_normal,
              const typename super_t::_point_t &point) {
        square     = _normal.norm();
        if (!std::isnormal(square))
            square = -1;
        else {
            float       t = 0;
            for (size_t i = 0; i < dim; ++i) {
                if (t != 0) {
                    normal[i] = _normal[i] / t;
                } else if (std::abs(_normal[i]) > float_equal) {
                    t = _normal[i];
                    normal[i] = 1;
                } else {
                    normal[i] = 0;
                }
            }
            
            square = normal.norm();
            b      = -(point * normal);
        }
    }
    
    BOTH float operator()(const typename super_t::_point_t &point) const {
        return square > 0 ? std::abs(normal * point + b) / square
                          : NAN;
    }
    
    BOTH bool is_valid() const { return square > 0; }
    
    BOTH bool operator==(const _plane_t &others) const {
        return (normal - others.normal).norm(1) < float_equal
               && std::abs(b - others.b) < float_equal;
    }
    
    BOTH bool operator!=(const _plane_t &others) const {
        return !operator==(others);
    }
    
    std::vector<float> parameters() const {
        std::vector<float> result(dim + 1);
        std::copy(normal.values, normal.values + dim, result.begin());
        result[dim - 1] = b;
        return result;
    }

    BOTH void parameters(float *dest) const {
        std::copy(normal.values, normal.values + dim, dest);
        dest[dim - 1] = b;
    }
};

template<size_t _dim>
struct plane_t {
    using super_t = model_t<_dim, _dim>;
};

template<>
struct plane_t<3> {
private:
    _plane_t<3> core;

public:
    using super_t = model_t<3, 3>;
    
    void make(const std::array<typename super_t::_point_t,
              super_t::size_to_make> &points) {
        const auto t0 = points[2] - points[0],
                   t1 = points[1] - points[0];
        core.make(point_t<3>{+t0.y() * t1.z() - t1.y() * t0.z(),
                             -t0.x() * t1.z() + t1.x() * t0.z(),
                             +t0.x() * t1.y() - t1.x() * t0.y()},
                  points[0]);
    }
    
    BOTH float operator()(const typename super_t::_point_t &point) const {
        return core(point);
    }
    
    BOTH bool is_valid() const {
        return core.is_valid();
    }
    
    BOTH bool operator==(const plane_t &others) const {
        return core == others.core;
    }
    
    BOTH bool operator!=(const plane_t &others) const {
        return core != others.core;
    }
    
    BOTH std::vector<float> parameters() const {
        return core.parameters();
    }
};

template<size_t _dim>
std::ostream &operator<<(std::ostream &ostream, const plane_t<_dim> &plane) {
    size_t i          = 0;
    auto   parameters = plane.parameters();
    while (i < _dim - 1) {
        if (parameters[i] != 0) {
            if (parameters[i] != 1)
                ostream << parameters[i] << ' ';
            ostream << 'x' << i << " + ";
        }
        ++i;
    }
    ostream << parameters[i] << " == 0";
    
    return ostream;
}


#endif // RANSAC_PLANE_T_HPP
