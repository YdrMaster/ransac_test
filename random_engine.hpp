//
// Created by User on 2019/6/4.
//

#ifndef RANSAC_TEST_RANDOM_ENGINE_HPP
#define RANSAC_TEST_RANDOM_ENGINE_HPP


#include <random>

template<class t>
class random_engine {
    static_assert(
        std::is_integral<t>::value || std::is_floating_point<t>::value,
        "t must be an numeric type");

public:
    constexpr static auto
        max_possible = std::is_integral<t>::value;

private:
    using uniform_distribution_t =
    typename std::conditional<
        max_possible,
        std::uniform_int_distribution<t>,
        std::uniform_real_distribution<t>
    >::type;
    
    std::mt19937           generator;
    uniform_distribution_t uniform_distribution;

public:
    explicit random_engine(t min, t max)
        : generator(std::random_device()()),
          uniform_distribution(min, max) {}
    
    t operator()() {
        return uniform_distribution(generator);
    }
};


#endif //RANSAC_TEST_RANDOM_ENGINE_HPP
