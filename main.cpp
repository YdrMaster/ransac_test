#include <iostream>
#include "ransac.h"

int main() {
    std::vector<line_t::_point_t> points{
        {{0, 0}},
        {{1, 1}},
        {{2, 2}},
        {{3, 6}},
        {{4, 4}},
        {{5, 5}},
    };
    
    auto result = ransac<line_t>(points, .1f, 10);
    
    std::cout << result.model.a() << "x + "
              << result.model.b() << "y + "
              << result.model.c() << " = 0" << std::endl;
    
    for (auto i : result.inliers)
        std::cout << points[i].x() << ", " << points[i].y() << std::endl;
    
    return 0;
}
