#include "VND.h"

VND::VND() {
    neighborhoods_.push_back(&LocalSearches::IR_1opt);                          // [0]
    neighborhoods_.push_back(&LocalSearches::IR_Exchange);                      // [1]
    neighborhoods_.push_back(&LocalSearches::IR_2opt);                          // [2]

    neighborhoods_.push_back(&LocalSearches::UN_1opt);                          // [3]
    neighborhoods_.push_back(&LocalSearches::UN_Exchange);                      // [4]

    neighborhoods_.push_back(&LocalSearches::ER_1opt);                          // [5]
    neighborhoods_.push_back(&LocalSearches::ER_Exchange_in_best_pos);          // [6]
    neighborhoods_.push_back(&LocalSearches::ER_2opt);                          // [7]
    neighborhoods_.push_back(&LocalSearches::ER_General_exchange);              // [8]

    neighborhoods_.push_back(&LocalSearches::ER_Synced_reinsertion);            // [9]
    neighborhoods_.push_back(&LocalSearches::ER_Parallel_exchange_in_best_pos); // [10]

    kmax_ = static_cast<int>(neighborhoods_.size());
}

bool VND::solve(Solution & solution, LS_strat search_strat) {
    int k = 0;
    bool curr_result, final_result = false;
    while (k < kmax_) {
        curr_result = (ls_.*neighborhoods_[k])(solution, search_strat);

        if (curr_result) {
            final_result = true;
            k = 0;
        } else {
            ++k;
        }
    }
    return final_result;
}
