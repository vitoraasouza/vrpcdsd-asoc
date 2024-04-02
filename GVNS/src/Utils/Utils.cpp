#include "Utils.h"

MyClock general_clock;

namespace rand_utils {
    uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator(seed);
}

int cmp(double x, double y, double tol) {
   return ( x <= y + tol ) ? ( x + tol < y ) ? -1 : 0 : 1;
}
