#ifndef VND_H_
#define VND_H_

#include <iostream>
#include <vector>

#include "../LocalSearches/LocalSearches.h"

using namespace std;

class VND {
    public:
        VND();

        bool solve(Solution & solution, LS_strat search_strat);

    private:
        LocalSearches ls_;
        vector<bool (LocalSearches::*)(Solution &, LS_strat)> neighborhoods_;
        int kmax_;
};


#endif /* VND_H_ */
