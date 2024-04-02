#include "InputSettings.h"

void InputSettings::parseSettings(const int argc, char * argv[]) {
    for (int i = 1; i < argc; i += 2) {
        if (strcmp(argv[i], "-i") == 0) {
            strcpy(instance_file_name_and_dir_, argv[i + 1]);
        } else if (strcmp(argv[i], "-t") == 0) {
            time_limit_ = atoi(argv[i + 1]);
        } else if (strcmp(argv[i], "-l") == 0) {
            strcpy(log_file_name_, argv[i + 1]);
        } else if (strcmp(argv[i], "-s") == 0) {
            strcpy(solution_file_name_, argv[i + 1]);
        } else if (strcmp(argv[i], "-tikz") == 0) {
            strcpy(tikz_file_name_, argv[i + 1]);
        } else if (strcmp(argv[i], "-strat") == 0) {
            if (strcmp(argv[i + 1], "first") == 0) {
                ls_strat_ = LS_strat::one_first_improv_;
            } else if (strcmp(argv[i + 1], "best") == 0) {
                ls_strat_ = LS_strat::one_best_improv_;
            }
        }
    }
}
