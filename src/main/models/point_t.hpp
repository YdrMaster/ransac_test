//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_POINT_T_HPP
#define RANSAC_POINT_T_HPP


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
    
    float &operator[](size_t i) { return values[i]; }
    
    float operator[](size_t i) const { return values[i]; }
    
    void fill(float value) { std::fill(values, values + dim, value); }
    
    bool is_valid() const {
        return std::find(values, values + dim, [](float value) { return std::isfinite(value); }) != values + dim;
    }
    
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
    
    point_t &operator*=(float k) {
        for (size_t i = 0; i < dim; ++i)
            values[i] *= k;
        return *this;
    }
    
    point_t &operator/=(float k) {
        for (size_t i = 0; i < dim; ++i)
            values[i] /= k;
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
    
    point_t operator*(const float &k) const {
        point_t result = *this;
        return result *= k;
    }
    
    point_t operator/(const float &k) const {
        point_t result = *this;
        return result /= k;
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
            for (auto item : values) result = std::fmaxf(std::abs(item), result);
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
        ostream << point[i++] << ", ";
    ostream << point[i] << ")";
    return ostream;
}


#endif //RANSAC_POINT_T_HPP
