#ifndef UTILS_H_
#define UTILS_H_

#include <chrono>
#include <random>
#include <vector>

int cmp(double x, double y, double tol = 1e-6);

#define time_now() chrono::high_resolution_clock::now()
#define duration(x) chrono::duration<double>(x).count()

class MyClock {
    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> start_tp_;
        double accumulated_time_;
        bool is_stopped_;

    public:
        MyClock() {
            accumulated_time_ = 0.0;
            is_stopped_ = true;
        }

        void start() {
            if (is_stopped_) {
                start_tp_ = std::chrono::high_resolution_clock::now();
                is_stopped_ = false;
            }
        }

        void stop() {
            if (!is_stopped_) {
                accumulated_time_ += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_tp_).count();
                is_stopped_ = true;
            }
        }

        void reset() {
            accumulated_time_ = 0.0;
            is_stopped_ = true;
        }

        double elapsed_time() {
            if (is_stopped_) {
                return accumulated_time_;
            } else {
                return accumulated_time_ + std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_tp_).count();
            }
        }
};

extern MyClock general_clock;

namespace rand_utils {
    extern uint64_t seed;
    extern std::mt19937 generator;
}

#define uni_rand(a,b) std::uniform_int_distribution<int>(a,b)(rand_utils::generator)

#endif /* UTILS_H_ */
