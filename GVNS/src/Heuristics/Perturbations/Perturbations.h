#ifndef PERTURBATIONS_H_
#define PERTURBATIONS_H_

#include <limits>
#include <vector>
#include "../Constructive/ConstructiveHeuristics.h"
#include "../../Solution/Solution.h"
#include "../LocalSearches/LocalSearches.h"

struct RCUnit {
    int node_, orig_vhc_, orig_pos_, dest_vhc_, dest_pos_;

    RCUnit(int node, int orig_vhc, int orig_pos) {
        node_ = node;
        orig_vhc_ = orig_vhc;
        orig_pos_ = orig_pos;
        dest_vhc_ = -1;
        dest_pos_ = -1;
    }
};

struct Perturbations {
    Perturbations();

    // IR - Multiplos exchanges / quantidade que vai aumentando / uma vez que um node eh retirado do seu lugar original, ele nao pode voltar mais
    void IR_Multiple_exchanges(Solution & solution);

    void UN_Multiple_exchanges(Solution & solution);

    // Primeiramente, tenta alinhar um pic-del usando um movimento de reinsercao. Caso nao consiga, tenta novamente com um movimento exchange
    void ER_Alignment(Solution & solution);

    void ER_Misalignment(Solution & solution);

    void ER_Mix_cyclic(Solution & solution);

    // variables
    vector<vector<int>> edge_penalties_;
    vector<int> load_penalties_;
    double lambda_;
};

#endif /* PERTURBATIONS_H_ */
