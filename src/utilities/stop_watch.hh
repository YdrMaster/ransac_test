//
// Created by user on 6/12/19.
//

#ifndef RANSAC_STOP_WATCH_HH
#define RANSAC_STOP_WATCH_HH


#include <chrono>
#include <atomic>

class stop_watch {
    std::atomic<decltype(std::chrono::steady_clock::now())> time;

public:
    stop_watch();
    
    double seconds() const;
    
    void reset();
};


#endif //RANSAC_STOP_WATCH_HH
