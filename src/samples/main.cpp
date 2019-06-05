#include <iostream>
#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

constexpr size_t n = 2000000;

int main() {
    std::vector<plane_t<3>::_point_t> points(n);
    
    random_engine<float>
        engine(-10, 10),
        noise(-.1, .1);
    
    while (true) {
        for (auto j = 0; j < n; ++j) {
            auto x = engine(),
                 y = engine(),
                 z = j % 2 ? -5 - 2 * x - 3 * y + noise()
                           : engine();
            points[j] = point_t<3>{x, y, z};
        }
        
        auto result = ransac<plane_t<3>>(points, .1f, 1.0f / 3, 20);
        
        std::cout << result.rate << std::endl;
        std::cout << result.model.normal << std::endl;
    }
    
    return 0;
}
