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

public:
    std::array<__point_t, super_t::size_to_make> points;
    
    void make(const std::array<__point_t, super_t::size_to_make> &points) final {
        this->points = points;
    }
    
    float operator()(const __point_t &point) const final {
        auto connect0 = point - points[0],
             connect1 = points[1] - points[0];
        auto dot      = connect0 * connect1;
        return std::sqrt(connect0.square() + dot * dot / connect1.square());
    }
    
    bool is_valid() const final {
        return points[0] != points[1];
    }
    
    bool operator==(const line_t &others) const {
        return others(points[0]) < 1E-6 && others(points[1]) < 1E-6;
    }
    
    bool operator!=(const line_t &others) const {
        return !operator==(others);
    }
};


#endif //RANSAC_LINE_T_HPP
