//
// Created by user on 6/12/19.
//

#include "stop_watch.hh"

using namespace std::chrono;

stop_watch::stop_watch() : time(steady_clock::now()) {}

double stop_watch::seconds() const {
    return duration_cast<duration<double, std::ratio<1>>>(steady_clock::now() - time.load()).count();
}

void stop_watch::reset() {
    time = steady_clock::now();
}
