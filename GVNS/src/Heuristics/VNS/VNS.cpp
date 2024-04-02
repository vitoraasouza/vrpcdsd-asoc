#include "VNS.h"

VNS::VNS() : vnd_() {
    neighborhoods_.push_back(&Perturbations::IR_Multiple_exchanges);
    neighborhoods_.push_back(&Perturbations::UN_Multiple_exchanges);
    neighborhoods_.push_back(&Perturbations::ER_Alignment);
    neighborhoods_.push_back(&Perturbations::ER_Misalignment);
    neighborhoods_.push_back(&Perturbations::ER_Mix_cyclic);

    kmax_ = static_cast<int>(neighborhoods_.size());
    time_to_find_best_sol_ = -1.0;
}

bool VNS::solve_GVNS(Solution & solution, InputSettings & input_settings) {

    time_to_find_best_sol_ = general_clock.elapsed_time();

    int k, outer_iter = 0, inner_iter = 0;
    bool internal_result, final_result = false;
    Solution best_sol = solution;
    double ptb_cost;
    int num_iter_without_improv = 0;
    int max_iter_without_improv = Instance::instance()->num_customers();

    FILE * log_file;
    log_file = fopen(input_settings.log_file_name_, "w");

    fprintf(log_file, "   %6s    %6s    %9s    %4s    %9s    %9s\n", "Outer", "Inner", "Perturbed", "", "Current", "Best");
    fprintf(log_file, "   %6s    %6s    %9s    %4s    %9s    %9s\n\n", "Iter", "Iter", "Solution", "Ptb", "Solution", "Solution");
    fprintf(log_file, "   %6d    %6d    %9s    %4s    %9.3lf    %9.3lf\n", outer_iter++, inner_iter++, "", "", solution.cost_, best_sol.cost_);

    for (;cmp(general_clock.elapsed_time(), input_settings.time_limit_) < 0 && num_iter_without_improv < max_iter_without_improv; ++outer_iter) {
        internal_result = false;
        k = 0;
        for ( ; k < kmax_; ++inner_iter) {
            (perturb_.*neighborhoods_[k])(solution);

            ptb_cost = solution.cost_;

            LocalSearches ls;
            if (k > 1) {
                vnd_.solve(solution, input_settings.ls_strat_);
            } else {
                ls.ER_General_exchange(solution, LS_strat::complete_first_improv_);
                vnd_.solve(solution, input_settings.ls_strat_);
            }

            if (cmp(solution.cost_, best_sol.cost_) < 0) {
                time_to_find_best_sol_ = general_clock.elapsed_time();
                best_sol = solution;
                fprintf(log_file, "%c  %6d    %6d    %9.3lf    [%2d]    %9.3lf    %9.3lf\n", '*', outer_iter, inner_iter, ptb_cost, k, solution.cost_, best_sol.cost_);
                internal_result = true;
                k = 0;
            } else {
                fprintf(log_file, "%c  %6d    %6d    %9.3lf    [%2d]    %9.3lf    %9.3lf\n", ' ', outer_iter, inner_iter, ptb_cost, k, solution.cost_, best_sol.cost_);
                ++k;
            }

            solution = best_sol;
        }

        if (internal_result) {
            final_result = true;
            num_iter_without_improv = 0;
        } else {
            ++num_iter_without_improv;
        }
    }

    return final_result;
}

double VNS::time_to_find_best_sol() {
    return time_to_find_best_sol_;
}
