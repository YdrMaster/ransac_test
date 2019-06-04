#include <iostream>
#include <vector>
#include <random>

#include "model.hpp"
#include "random_engine.hpp"

// 迭代参数
constexpr auto P = 0.3f; // 判断模型是否适用于数据的比例

template<class _model_t>
void ransac(const std::vector<typename _model_t::_point_t> &data,
            float threshold,
            unsigned int max_times) {
    using tp = typename _model_t::_point_t;
    using ti = decltype(data.size());
    
    if (data.size() < 2 * _model_t::size_to_make)
        throw std::logic_error("samples too little");
    
    // 随机数引擎
    random_engine<decltype(data.size())>
        random(0, data.size() - 1);
    // 模型初始化器
    std::array<tp, _model_t::size_to_make>
        initialize_list{};
    // 缓存
    std::vector<bool>
        check_buffer(data.size());
    
    _model_t best_model{}, model{};
    size_t   best_count = 0;
    
    for (ti i = 0; i < max_times; ++i) {
        for (ti j = 0; j < initialize_list.size(); ++j)
            initialize_list[j] = data[random()];
        
        model.make(initialize_list);
        if (!model.valid() || (best_model.valid() && (model.normalize(), model == best_model)))
            continue;
        
        std::transform(data.begin(), data.end(),
                       check_buffer.begin(),
                       [&](const tp &point) {
                           return std::abs(model(point)) < threshold;
                       });
        
        size_t    count = 0;
        for (auto r : check_buffer)
            if (r) ++count;
        
        if (count > best_count) {
            best_model = model;
            best_count = count;
            best_model.normalize();
        }
    }
    
    std::cout << best_model.a() << " "
              << best_model.b() << " "
              << best_model.c() << " "
              << best_count << std::endl;
}

int main() {
    std::vector<line_t::_point_t> points{
        {{0, 0}},
        {{1, 1}},
        {{2, 2}},
        {{3, 6}},
        {{4, 4}},
        {{4, 4}},
    };
    
    ransac<line_t>(points, .1f, 10);
    
    return 0;
}
