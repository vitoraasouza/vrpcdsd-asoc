#ifndef INPUT_SETTINGS_H_
#define INPUT_SETTINGS_H_

#include <cstring>
#include <cstdlib>
#include "../Heuristics/LocalSearches/LocalSearches.h"

struct InputSettings {
    char instance_file_name_and_dir_[200], log_file_name_[200], solution_file_name_[200], tikz_file_name_[200];
    bool parameters_on_;
    double time_limit_;
    int num_threads_;
    LS_strat ls_strat_;

    void parseSettings(const int argc, char * argv[]);
};

#endif // INPUT_SETTINGS_H_
