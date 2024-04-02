#ifndef CONSTRUCTIVE_HEURISTICS_H_
#define CONSTRUCTIVE_HEURISTICS_H_

#include "../../Solution/Solution.h"
#include "../../Utils/Utils.h"
#include <map>
#include <set>
#include <iterator>

using namespace std;

struct ConstructiveHeuristics {
    void ExtendedSavings(Solution & solution);
    void CreateWeightedSavingsList(multimap<double, pair<int, int> > & saving_i_j, double capacity_weight, char pic_or_del);
    bool CreateRoutes(Routes & routes, multimap<double, pair<int, int> > & saving_i_j);
    void IncreaseNumberOfRoutes(Routes & routes);
    void AlignPickupAndDelivery(Solution & solution, Routes & pickup_routes, Routes & delivery_routes);
    void DefineScheduling(Solution & solution);
    void DefineScheduling2(Solution & solution);
};

#endif // CONSTRUCTIVE_HEURISTICS_H_
