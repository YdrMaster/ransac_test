//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_PLANE_T_HPP
#define RANSAC_PLANE_T_HPP


#include "model_t.hpp"

template<size_t _dim>
struct plane_t final : public model_t<_dim, _dim> {
private:
    static_assert(_dim >= 3, "plane dimension is at least 3");
    
    using super_t   = model_t<_dim, _dim>;
    using __point_t = typename super_t::_point_t;
    
    float _norm = -1;

public:
    std::array<float, _dim + 1> parameters{};
    
    void make(const std::array<__point_t, super_t::size_to_make> &points) final {
        float     temp = 0;
        for (auto item : parameters) temp += item * item;
        _norm = std::sqrtf(temp);
    }
    
    float operator()(const __point_t &point) const final {
        float       temp = 0;
        for (size_t i    = 0; i < _dim; ++i)
            temp += parameters[i] * point.values[i];
        return std::fabsf(temp + parameters.back()) / _norm;
    }
    
    bool is_valid() const final {
        return std::any_of(
            parameters.begin(),
            parameters.end() - 1,
            [](float value) { return value > float_equal; });
    }
    
    bool operator==(const plane_t &others) const {
        for (size_t i = 0; i < _dim + 1; ++i)
            if (std::fabsf(parameters[i] - others.parameters[i]) > float_equal)
                return false;
        return true;
    }
    
    bool operator!=(const plane_t &others) const {
        return !operator==(others);
    }
};


#endif // RANSAC_PLANE_T_HPP
