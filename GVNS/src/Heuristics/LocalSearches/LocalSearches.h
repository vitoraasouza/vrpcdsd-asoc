#ifndef LOCALSEARCHES_H_
#define LOCALSEARCHES_H_

#include "../../Persistent/Instance.h"
#include "../../Solution/Solution.h"
#include <cstdlib>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <map>
#include <numeric>
#include <set>
#include <unordered_map>
#include <utility>

enum LS_strat {
    complete_best_improv_,
    complete_first_improv_,
    one_best_improv_,
    one_first_improv_
};

struct LocalSearches {

    // Intra route local searches
    bool IR_1opt(Solution & solution, LS_strat search_strat);
    bool IR_2opt(Solution & solution, LS_strat search_strat);
    bool IR_Exchange(Solution & solution, LS_strat search_strat);

    double IR_1opt_pickup_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos);
    double IR_1opt_delivery_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos);
    void IR_move_1opt_pickup_route(Solution & solution, const int route, const int orig_pos, const int dest_pos);
    void IR_move_1opt_delivery_route(Solution & solution, const int route, const int orig_pos, const int dest_pos);

    double IR_2opt_pickup_delta_cost(Solution & solution, const int route, const int first_pos, const int last_pos);
    double IR_2opt_delivery_delta_cost(Solution & solution, const int route, const int first_pos, const int last_pos);
    void IR_move_2opt_pickup_route(Solution & solution, const int route, const int first_pos, const int last_pos);
    void IR_move_2opt_delivery_route(Solution & solution, const int route, const int first_pos, const int last_pos);

    double IR_exchange_pickup_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos);
    double IR_exchange_delivery_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos);
    void IR_move_exchange_pickup_route(Solution & solution, const int route, const int orig_pos, const int dest_pos);
    void IR_move_exchange_delivery_route(Solution & solution, const int route, const int orig_pos, const int dest_pos);

    void IR_update_schedule(Solution & solution, int route, double diff_cost, double & completion_diffs);

    // Unload local searches
    bool UN_1opt(Solution & solution, LS_strat search_strat);
    bool UN_Exchange(Solution & solution, LS_strat search_strat);

    double UN_1opt_delta_cost(Solution & solution, const int vhc, const int in_pos, const int out_pos);
    void UN_move_1opt(Solution & solution, const int k, const int selected_pos, int best_pos);

    double UN_exchange_delta_cost(Solution & solution, const int vhc, const int orig_pos, const int dest_pos);
    void UN_move_exchange(Solution & solution, const int k, const int selected_pos, int best_pos);

    // Inter route local searches
    bool ER_1opt(Solution & solution, LS_strat search_strat);
    bool ER_2opt(Solution & solution, LS_strat search_strat);
    bool ER_Exchange_in_best_pos(Solution & solution, LS_strat search_strat);

    int ER_1opt_best_insertion_pos(VisitedNodes & visited_nodes, const int insert_node);
    double ER_1opt_pickup_delta_cost(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, int & unl_pos);
    double ER_1opt_delivery_delta_cost(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, int & unl_pos);
    void ER_move_1opt_pickup_routes(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, const int unl_pos);
    void ER_move_1opt_delivery_routes(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, const int unl_pos);

    double ER_2opt_pickup_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov = false);
    double ER_2opt_delivery_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov = false);
    void ER_move_2opt_pickup_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov);
    void ER_move_2opt_delivery_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov);

    double ER_exchange_in_best_pos_pickup_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, int & final_pos_for_n1, int & final_pos_for_n2, int & unl_pos_r1, int & unl_pos_r2);
    double ER_exchange_in_best_pos_delivery_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, int & final_pos_for_n1, int & final_pos_for_n2, int & unl_pos_r1, int & unl_pos_r2);
    void ER_move_exchange_in_best_pos_pickup_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, const int final_pos_for_n1, const int final_pos_for_n2, const int unl_pos_r1, const int unl_pos_r2);
    void ER_move_exchange_in_best_pos_delivery_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, const int final_pos_for_n1, const int final_pos_for_n2, const int unl_pos_r1, const int unl_pos_r2);

    // Advanced local searches
    bool ER_Parallel_exchange_in_best_pos(Solution & solution, LS_strat search_strat);
    bool ER_Synced_reinsertion(Solution & solution, LS_strat search_strat);
    bool ER_General_exchange(Solution & solution, LS_strat search_strat);

    double ER_parallel_exchange_in_best_pos_delta_cost(Solution & solution, const int k1_pic, const int pos_r1_pic, const int k1_del, const int pos_r1_del,
                                                       const int k2_pic, const int pos_r2_pic, const int k2_del, const int pos_r2_del,
                                                       int & fpos_r1_pic, int & fpos_r2_pic, int & fpos_r1_del, int & fpos_r2_del,
                                                       int & unl_pos_r1, int & unl_pos_r2);
    void ER_move_parallel_exchange_in_best_pos(Solution & solution, const int k1_pic, const int pos_r1_pic, const int k1_del, const int pos_r1_del,
                                               const int k2_pic, const int pos_r2_pic, const int k2_del, const int pos_r2_del,
                                               const int fpos_r1_pic, const int fpos_r2_pic, const int fpos_r1_del, const int fpos_r2_del,
                                               const int unl_pos_r1, const int unl_pos_r2);

    double ER_Synced_reinsertion_delta_cost(Solution & solution, const int k1_pic, const int pos_k1_pic, const int k1_del, const int pos_k1_del,
                                            const int k2_pic, const int pos_k2_pic, const int k2_del, const int pos_k2_del,
                                            int & unl_pos);
    void ER_move_synced_reinsertion(Solution & solution, const int k1_pic, const int pos_k1_pic, const int k1_del, const int pos_k1_del,
                                    const int k2_pic, const int pos_k2_pic, const int k2_del, const int pos_k2_del,
                                    const int unl_pos);

    double ER_general_exchange_pickup_delta_cost(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                 map<int, vector<int>> & vhc_unload_order, bool & k1_invert, bool & k2_invert);
    double ER_general_exchange_delivery_delta_cost(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                   map<int, vector<int>> & vhc_unload_order, bool & k1_invert, bool & k2_invert);
    void ER_move_general_exchange_pickup_routes(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                map<int, vector<int>> & vhc_unload_order, const bool k1_invert, const bool k2_invert);
    void ER_move_general_exchange_delivery_routes(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                  map<int, vector<int>> & vhc_unload_order, const bool k1_invert, const bool k2_invert);
};

#endif // LOCALSEARCHES_H_
