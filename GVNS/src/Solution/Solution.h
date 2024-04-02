#ifndef SOLUTION_H_
#define SOLUTION_H_

#include <cmath>
#include <vector>

#include "Route.h"
#include "ScheduleManager.h"

using namespace std;

struct Solution {

    Routes routes_pickup_, routes_delivery_;
    ScheduleManager schedule_manager_;
    int num_vehicles_;
    double cost_;

    Solution(const int num_requests, const int num_vehicles);

    void printTikZSolutionGuide(const char * tikz_guide_file_name);
    bool checkFeasibility();

    Solution & operator=(const Solution & solution);
    friend ostream & operator<<(ostream & out, const Solution & solution);

};

#endif /* SOLUTION_H_ */
