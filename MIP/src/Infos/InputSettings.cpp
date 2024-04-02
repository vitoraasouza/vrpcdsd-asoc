#include "InputSettings.h"

InputSettings::InputSettings() {
    strcpy(instance_file_name_and_dir_, "");
    strcpy(log_file_name_, "log");
    strcpy(solution_file_name_, "sol");
    strcpy(tikz_file_name_, "tikz");
    strcpy(tikz_relaxation_file_name_, "rel");
    parameters_on_ = true;
    only_relaxation_ = false;
    time_limit_ = 0.0;
    num_threads_ = 0; // CPLEX decides
    obj_ = UNKNOWN_;
}

bool InputSettings::parseSettings(const int argc, char * argv[]) {
    bool filled_instance = false, filled_obj = false, filled_time = false;

    for (int i = 1; i < argc; i += 2) {
        if (strcmp(argv[i], "-i") == 0) {
            strcpy(instance_file_name_and_dir_, argv[i + 1]);
            filled_instance = true;
        } else if (strcmp(argv[i], "-t") == 0) {
            time_limit_ = atoi(argv[i + 1]);
            filled_time = true;
        } else if (strcmp(argv[i], "-p") == 0) {
            if (strcmp(argv[i + 1], "ON") == 0) {
                parameters_on_ = true;
            } else {
                parameters_on_ = false;
            }
        } else if (strcmp(argv[i], "-l") == 0) {
            strcpy(log_file_name_, argv[i + 1]);
        } else if (strcmp(argv[i], "-s") == 0) {
            strcpy(solution_file_name_, argv[i + 1]);
        } else if (strcmp(argv[i], "-th") == 0) {
            num_threads_ = atoi(argv[i + 1]);
        } else if (strcmp(argv[i], "-tikz") == 0) {
            strcpy(tikz_file_name_, argv[i + 1]);
        } else if (strcmp(argv[i], "-c") == 0) {
            if (strcmp(argv[i + 1], "TCT") == 0) {
                obj_ = TOTAL_COMPLETION_TIME_;
                filled_obj = true;
            } else if (strcmp(argv[i + 1], "MKP") == 0) {
                obj_ = MAKESPAN_;
                filled_obj = true;
            }
        } else if (strcmp(argv[i], "-rel") == 0) {
            strcpy(tikz_relaxation_file_name_, argv[i + 1]);
            only_relaxation_ = true;
        }
    }

    if (!filled_instance) {
        cerr << "Instance name not inserted!" << endl;
    }
    if (!filled_obj) {
        cerr << "Criterion not inserted!" << endl;
    }
    if (!filled_time) {
        cerr << "Time limit not inserted!" << endl;
    }

    return filled_instance && filled_obj && filled_time;
}
