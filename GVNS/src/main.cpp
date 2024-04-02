#include "Infos/InputSettings.h"
#include "Persistent/Instance.h"
#include "Solution/Solution.h"
#include "Heuristics/Constructive/ConstructiveHeuristics.h"
#include "Heuristics/LocalSearches/LocalSearches.h"
#include "Heuristics/Perturbations/Perturbations.h"
#include "Heuristics/VND/VND.h"
#include "Heuristics/VNS/VNS.h"
#include "Utils/Utils.h"
#include <fstream>

void printReport(Solution & solution, VNS & vns) {
    printf("%s;%.3lf;%.10lf;%.10lf;%lu\n", Instance::instance()->name(), solution.cost_, vns.time_to_find_best_sol(), general_clock.elapsed_time(), rand_utils::seed);
}

void printSolutionOnFile(Solution & solution, InputSettings & input_settings) {
    ofstream sol_file(input_settings.solution_file_name_);
    sol_file << solution << endl;
    sol_file.close();
}

int main(int argc, char * argv[]) {

    cout.precision(3);
    cout << fixed;

    InputSettings input_settings;
    input_settings.parseSettings(argc, argv);

    Instance::Init(input_settings.instance_file_name_and_dir_);

    general_clock.start();

    Solution solution(Instance::instance()->num_requests(), Instance::instance()->num_vehicles());

    ConstructiveHeuristics ch;
    VNS vns;

    ch.ExtendedSavings(solution);
    vns.solve_GVNS(solution, input_settings);

//    cout << endl << solution << endl;

    general_clock.stop();

//    if (solution.checkFeasibility()) {
//        cout << "SOLUCAO VIAVEL!" << endl;
//    } else {
//        cout << "SOLUCAO INVIAVEL!" << endl;
//    }

    solution.printTikZSolutionGuide(input_settings.tikz_file_name_);

    printSolutionOnFile(solution, input_settings);

    printReport(solution, vns);

    return 0;
}
