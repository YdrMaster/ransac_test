//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_TEST_MODEL_HPP
#define RANSAC_TEST_MODEL_HPP


#include <vector>
#include <array>

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

/**
 * 模型
 *
 * @tparam _parameter_dim 参数维度
 * @tparam _point_dim     点维度
 * @tparam _size_to_make  确定模型所需最小点数
 */
template<
    size_t _parameter_dim,
    size_t _point_dim,
    size_t _size_to_make
> struct model_t {
    using _point_t = point_t<_point_dim>;
    
    constexpr static auto
        parameter_dim = _parameter_dim,
        point_dim     = _point_dim,
        size_to_make  = _size_to_make;
    
    std::array<float, _parameter_dim> parameters;
    
    model_t() { parameters.fill(NAN); }
    
    virtual void make(const std::array<_point_t, size_to_make> &) = 0;
    
    virtual float operator()(const _point_t &) const = 0;
    
    bool valid() const {
        return !std::any_of(
            parameters.begin(), parameters.end(),
            [](float value) { return std::isnan(value); });
    }
    
    void normalize() {
        for (size_t i = 0; i < parameter_dim; ++i) {
            if (parameters[i] != 0) {
                for (auto j   = i + 1; j < parameter_dim; ++j)
                    parameters[j] /= parameters[i];
                parameters[i] = 1;
                return;
            }
        }
    }
    
    bool operator==(const model_t &others) const {
        for (auto i = 0; i < parameter_dim; ++i)
            if (parameters[i] != others.parameters[i])
                return false;
        return true;
    }
    
    bool operator!=(const model_t &others) const {
        return !operator==(others);
    }
};

struct line_t final : public model_t<3, 2, 2> {
    
    float &a() { return parameters[0]; }
    
    float &b() { return parameters[1]; }
    
    float &c() { return parameters[2]; }
    
    float a() const { return parameters[0]; }
    
    float b() const { return parameters[1]; }
    
    float c() const { return parameters[2]; }
    
    void make(const std::array<_point_t, size_to_make> &points) final {
        if (points[0].x() == points[1].x()) {
            if (points[0].y() == points[1].y())
                a() = b() = c() = NAN;
            else
                a() = 1, b() = 0, c() = -points[0].x();
        } else {
            if (points[0].y() == points[1].y())
                a() = 0, b() = 1, c() = -points[0].y();
            else
                a() = points[1].y() - points[0].y(),
                b() = points[0].x() - points[1].x(),
                c() = points[0].y() * points[1].x() - points[1].y() * points[0].x();
        }
    }
    
    float operator()(const point_t<2> &p) const final {
        return std::abs(a() * p.x() + b() * p.y() + c()) / std::hypot(a(), b());
    }
};


#endif //RANSAC_TEST_MODEL_HPP
