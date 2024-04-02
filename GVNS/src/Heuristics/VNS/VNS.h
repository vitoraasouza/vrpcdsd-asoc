#ifndef VNS_H_
#define VNS_H_

#include "../../Infos/InputSettings.h"
#include "../Perturbations/Perturbations.h"
#include "../VND/VND.h"

class VNS {
    public:
        VNS();

        bool solve_GVNS(Solution & solution, InputSettings & input_settings);

        double time_to_find_best_sol();

    private:
        VND vnd_;
        Perturbations perturb_;

        vector<void (Perturbations::*)(Solution &)> neighborhoods_;
        int kmax_;
        double time_to_find_best_sol_;
};

#endif /* VNS_H_ */
