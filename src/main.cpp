#include <iostream>
#include "main/ransac.h"
#include "main/models/line_t.hpp"

int main() {
    std::vector<line_t<2>::_point_t> points{
        {{0, 0}},
        {{1, 1}},
        {{2, 2}},
        {{3, 6}},
        {{4, 4}},
        {{5, 5}},
    };
    
    auto result = ransac<line_t<2>>(points, .1f, 10);
    
    for (auto i : result.inliers)
        std::cout << points[i] << std::endl;
    
    return 0;
}
