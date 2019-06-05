#include <iostream>
#include "../main/ransac.h"
#include "../main/models/line_t.hpp"
#include "../main/models/plane_t.hpp"

int main() {
    std::vector<plane_t<3>::_point_t> points{};
    
    random_engine<float> engine(-10, 10);
    
    for (auto j = 0; j < 10000; ++j) {
        auto x = engine(),
             y = engine(),
             z = j % 4 ? -5 - 2 * x - 3 * y : engine();
        points.push_back(point_t<3>{x, y, z});
    }
    
    auto result = ransac<plane_t<3>>(points, .1f, 0.6f);
    
    std::cout << result.inliers.size() << std::endl;
    std::cout << result.model.normal << std::endl;
    
    return 0;
}
