//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_LINE_T_HPP
#define RANSAC_LINE_T_HPP


#include "model_t.hpp"

template<size_t _dim>
struct line_t final : public model_t<_dim, 2> {
private:
    using super_t   = model_t<_dim, 2>;
    using __point_t = typename super_t::_point_t;
    
    constexpr static auto float_equal = 1E-6f;
    
    float _square = 1;

public:
    __point_t point0{}, point1{}, connect{};
    
    void make(const std::array<__point_t, super_t::size_to_make> &points) final {
        point0  = points[0];
        point1  = points[1];
        connect = points[1] - points[0];
        _square = connect.square();
    }
    
    float operator()(const __point_t &point) const final {
        auto connect0 = point - point0;
        if (connect0.norm(1) < float_equal) return 0;
        auto dot = connect0 * connect;
        return std::sqrt(connect0.square() - dot * dot / _square);
    }
    
    bool is_valid() const final { return _square > float_equal; }
    
    bool operator==(const line_t &others) const {
        return others(point0) < float_equal && others(point1) < float_equal;
    }
    
    bool operator!=(const line_t &others) const {
        return !operator==(others);
    }
};


#endif //RANSAC_LINE_T_HPP
