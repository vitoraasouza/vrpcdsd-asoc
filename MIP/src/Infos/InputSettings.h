#ifndef INPUT_SETTINGS_H_
#define INPUT_SETTINGS_H_

#include <cstring>
#include <cstdlib>
#include <iostream>

using namespace std;

enum Criterion {
    UNKNOWN_ = 0,
    TOTAL_COMPLETION_TIME_ = 1,
    MAKESPAN_ = 2
};

struct InputSettings {
    char instance_file_name_and_dir_[200], log_file_name_[200], solution_file_name_[200], tikz_file_name_[200], tikz_relaxation_file_name_[200];
    bool parameters_on_, only_relaxation_;
    double time_limit_;
    int num_threads_;
    Criterion obj_;

    InputSettings();
    bool parseSettings(const int argc, char * argv[]);
};

#endif // INPUT_SETTINGS_H_
