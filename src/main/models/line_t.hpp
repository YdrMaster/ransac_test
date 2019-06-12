//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_LINE_T_HPP
#define RANSAC_LINE_T_HPP


#include <array>
#include <cmath>

#include "macro.h"
#include "model_t.hpp"

template<size_t _dim>
struct line_t {
private:
    static_assert(_dim >= 2, "line dimension is at least 2");
    
    float _square = -1;

public:
    using super_t = model_t<_dim, 2>;
    
    typename super_t::_point_t
        point0{},
        point1{},
        connect{};
    
    void make(const std::array<typename super_t::_point_t,
              super_t::size_to_make> &points) {
        point0  = points[0];
        point1  = points[1];
        connect = points[1] - points[0];
        _square = connect.square();
    }
    
    BOTH float operator()(const typename super_t::_point_t &point) const {
        auto connect0 = point - point0;
        if (connect0.norm(1) < float_equal) return 0;
        auto dot = connect0 * connect;
        return std::sqrt(connect0.square() - dot * dot / _square);
    }
    
    BOTH bool is_valid() const { return _square > 0; }
    
    BOTH bool operator==(const line_t &others) const {
        return others(point0) < float_equal && others(point1) < float_equal;
    }
    
    BOTH bool operator!=(const line_t &others) const {
        return !operator==(others);
    }
};


#endif //RANSAC_LINE_T_HPP
