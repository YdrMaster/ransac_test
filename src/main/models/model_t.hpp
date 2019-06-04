//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_MODEL_T_HPP
#define RANSAC_MODEL_T_HPP


#include <vector>
#include <array>
#include <cmath>

template<size_t _dim>
struct point_t {
    float values[_dim]{};
    
    constexpr static auto dim = _dim;
    
    float &x() { return values[0]; }
    
    float &y() { return values[1]; }
    
    float &z() { return values[2]; }
    
    float x() const { return values[0]; }
    
    float y() const { return values[1]; }
    
    float z() const { return values[2]; }
    
    point_t &operator+=(const point_t &others) {
        for (size_t i = 0; i < dim; ++i)
            values[i] += others.values[i];
        return *this;
    }
    
    point_t &operator-=(const point_t &others) {
        for (size_t i = 0; i < dim; ++i)
            values[i] -= others.values[i];
        return *this;
    }
    
    point_t operator+(const point_t &others) const {
        point_t result = *this;
        return result += others;
    }
    
    point_t operator-(const point_t &others) const {
        point_t result = *this;
        return result -= others;
    }
    
    float operator*(const point_t &others) const {
        float       result = 0;
        for (size_t i      = 0; i < dim; ++i)
            result += values[i] * others.values[i];
        return result;
    }
    
    float square() const {
        float     result = 0;
        for (auto item : values) result += item * item;
        return result;
    }
    
    float norm(float n = 2) const {
        if (std::isnan(n) || n <= 0) throw std::logic_error("n must be positive");
        
        float result = 0;
        if (std::isinf(n)) {
            for (auto item : values) result = std::max(std::abs(item), result);
        } else {
            for (auto item : values) result += std::pow(std::abs(item), n);
            result = std::pow(result, 1 / n);
        }
        return result;
    }
    
    bool operator==(const point_t &others) const {
        for (auto i = 0; i < _dim; ++i)
            if (values[i] != others.values[i])
                return false;
        return true;
    }
    
    bool operator!=(const point_t &others) const {
        return !operator==(others);
    }
};

template<size_t _dim>
std::ostream &operator<<(std::ostream &ostream, const point_t<_dim> &point) {
    ostream << "(";
    size_t i = 0;
    while (i < _dim - 1)
        ostream << point.values[i++] << ", ";
    ostream << point.values[i] << ")";
    return ostream;
}

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
    
    virtual void make(const std::array<_point_t, size_to_make> &) = 0;
    
    virtual float operator()(const _point_t &) const = 0;
    
    virtual bool is_valid() const = 0;
};


#endif //RANSAC_MODEL_T_HPP
