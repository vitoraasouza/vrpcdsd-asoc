#include "ConstructiveHeuristics.h"

void ConstructiveHeuristics::ExtendedSavings(Solution & solution) {

    Routes pickup_routes, delivery_routes;
    multimap<double, pair<int, int> > saving_i_j;

    double capacity_weight = 0.0;
    while (pickup_routes.empty() && capacity_weight <= 1.0) {
        CreateWeightedSavingsList(saving_i_j, capacity_weight, 'p');
        if (!CreateRoutes(pickup_routes, saving_i_j)) {
            pickup_routes.clear();
            saving_i_j.clear();
            capacity_weight += 0.1;
        }
    }

    saving_i_j.clear();
    capacity_weight = 0.0;

    while (delivery_routes.empty() && capacity_weight <= 1.0) {
        CreateWeightedSavingsList(saving_i_j, capacity_weight, 'd');
        if (!CreateRoutes(delivery_routes, saving_i_j)) {
            delivery_routes.clear();
            saving_i_j.clear();
            capacity_weight += 0.1;
        }
    }

    AlignPickupAndDelivery(solution, pickup_routes, delivery_routes);

    DefineScheduling2(solution);

    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution.cost_ += solution.routes_delivery_[k].cost_ + solution.schedule_manager_.schedules_[k].completion_time_;
    }

}

void ConstructiveHeuristics::CreateWeightedSavingsList(multimap<double, pair<int, int> > & saving_i_j, double capacity_weight, char pic_or_del) {
    int ini, end;
    if (pic_or_del == 'p') {
        ini = 1;
        end = ini + Instance::instance()->num_suppliers();
    } else {
        ini = Instance::instance()->num_suppliers() + 1;
        end = ini + Instance::instance()->num_customers();
    }
    for (int i = ini; i < end; ++i) {
        for (int j = i + 1; j < end; ++j) {
            saving_i_j.insert(make_pair((1.0 - capacity_weight) * (Instance::instance()->EdgeCost(i, j) - Instance::instance()->EdgeCost(i, 0) - Instance::instance()->EdgeCost(0, j))
                                        - capacity_weight * (Instance::instance()->node(i).demand() + Instance::instance()->node(j).demand()),
                                        make_pair(i,j)));
        }
    }
}

bool ConstructiveHeuristics::CreateRoutes(Routes & routes, multimap<double, pair<int, int> > & saving_i_j) {
    vector<bool> is_in_middle(2 * Instance::instance()->num_requests() + 1, false);
    vector<int> req_vhc(2 * Instance::instance()->num_requests() + 1, -1);
    int u, v, current_vhc, previous_vhc;
    for (auto it = saving_i_j.begin(); it != saving_i_j.end(); ++it) {
        u = it->second.first;
        v = it->second.second;

        if (!is_in_middle[u] && !is_in_middle[v]) {
            if (req_vhc[u] == -1 && req_vhc[v] == -1
                && Instance::instance()->node(u).demand() + Instance::instance()->node(v).demand() <= Instance::instance()->vehicle().capacity()) {

                current_vhc = routes.size();
                req_vhc[u] = req_vhc[v] = current_vhc;
                routes.resize(routes.size() + 1);
                routes[current_vhc].InsertVisitedNode(routes[current_vhc].last_insertable_pos_, u);
                routes[current_vhc].InsertVisitedNode(routes[current_vhc].last_insertable_pos_, v);
            } else if (req_vhc[u] == -1
                       && Instance::instance()->node(u).demand() + routes[req_vhc[v]].load_ <= Instance::instance()->vehicle().capacity()) {

                current_vhc = req_vhc[v];
                req_vhc[u] = current_vhc;
                if (v == routes[current_vhc].visited_nodes_[Route::first_insertable_pos_]) {
                    routes[current_vhc].InsertVisitedNode(Route::first_insertable_pos_, u);
                } else {
                    routes[current_vhc].InsertVisitedNode(routes[current_vhc].last_insertable_pos_, u);
                }
                is_in_middle[v] = true;
            } else if (req_vhc[v] == -1
                       && Instance::instance()->node(v).demand() + routes[req_vhc[u]].load_ <= Instance::instance()->vehicle().capacity()) {

                current_vhc = req_vhc[u];
                req_vhc[v] = current_vhc;
                if (u == routes[current_vhc].visited_nodes_[Route::first_insertable_pos_]) {
                    routes[current_vhc].InsertVisitedNode(Route::first_insertable_pos_, v);
                } else {
                    routes[current_vhc].InsertVisitedNode(routes[current_vhc].last_insertable_pos_, v);
                }
                is_in_middle[u] = true;
            } else if (req_vhc[u] != req_vhc[v] && req_vhc[u] != -1 && req_vhc[v] != -1
                       && routes[req_vhc[u]].load_ + routes[req_vhc[v]].load_ <= Instance::instance()->vehicle().capacity()) {
                current_vhc = req_vhc[u];
                previous_vhc = req_vhc[v];
                for (int i = 1; i <= routes[previous_vhc].num_nodes_; ++i) {
                    req_vhc[routes[previous_vhc].visited_nodes_[i]] = current_vhc;
                }
                if (u == routes[current_vhc].visited_nodes_[Route::first_insertable_pos_]) {
                    if (v == routes[previous_vhc].visited_nodes_[Route::first_insertable_pos_]) {
                        for (int i = 1; i <= routes[previous_vhc].num_nodes_; ++i) {
                            routes[current_vhc].InsertVisitedNode(Route::first_insertable_pos_, routes[previous_vhc].visited_nodes_[i]);
                        }
                    } else {
                        for (int i = routes[previous_vhc].num_nodes_; i >= 1; --i) {
                            routes[current_vhc].InsertVisitedNode(Route::first_insertable_pos_, routes[previous_vhc].visited_nodes_[i]);
                        }
                    }
                } else {
                    if (v == routes[previous_vhc].visited_nodes_[Route::first_insertable_pos_]) {
                        for (int i = 1; i <= routes[previous_vhc].num_nodes_; ++i) {
                            routes[current_vhc].InsertVisitedNode(routes[current_vhc].last_insertable_pos_, routes[previous_vhc].visited_nodes_[i]);
                        }
                    } else {
                        for (int i = routes[previous_vhc].num_nodes_; i >= 1; --i) {
                            routes[current_vhc].InsertVisitedNode(routes[current_vhc].last_insertable_pos_, routes[previous_vhc].visited_nodes_[i]);
                        }
                    }
                }
                routes[previous_vhc].Clear();
                is_in_middle[u] = is_in_middle[v] = true;
            }
        }
    }

    int begin_for, end_for;
    if (saving_i_j.begin()->second.first > Instance::instance()->num_suppliers()) { // verify if it's supplier or customer's side
        begin_for = Instance::instance()->num_suppliers() + 1;
        end_for = 2 * Instance::instance()->num_suppliers();
    } else {
        begin_for = 1;
        end_for = Instance::instance()->num_suppliers();
    }
    for (int r = begin_for; r <= end_for; ++r) {
        if (req_vhc[r] == -1) {
            for (uint vhc = 0; vhc < routes.size(); ++vhc) {
                if (routes[vhc].load_ + Instance::instance()->node(r).demand() <= Instance::instance()->vehicle().capacity()) {
                    req_vhc[r] = vhc;
                    routes[vhc].InsertVisitedNode(routes[vhc].last_insertable_pos_, r);
                }
            }

            if (req_vhc[r] == -1) {
                current_vhc = routes.size();
                routes.resize(routes.size() + 1);
                req_vhc[r] = current_vhc;
                routes[current_vhc].InsertVisitedNode(routes[current_vhc].last_insertable_pos_, r);
            }
        }
    }

    for (int i = routes.size() - 1; i >= 0; --i) {
        if (routes[i].num_nodes_ == 0) {
            routes.erase(routes.begin() + i);
        }
    }

    if (static_cast<int>(routes.size()) < Instance::instance()->num_vehicles()) {
        IncreaseNumberOfRoutes(routes);
    }

    return static_cast<int>(routes.size()) == Instance::instance()->num_vehicles();
}

void ConstructiveHeuristics::IncreaseNumberOfRoutes(Routes & routes) {
    int current_num_routes = routes.size();
    int orig_vhc = -1, orig_pos = -1;
    double reinsertion_cost, best_reinsertion_cost;

    routes.resize(Instance::instance()->num_vehicles());

    while (current_num_routes < Instance::instance()->num_vehicles()) {
        best_reinsertion_cost = 1000000000.0;
        for (int k = 0; k < current_num_routes; ++k) {
            if (routes[k].num_nodes_ > 1) {
                for (int i = Route::first_insertable_pos_; i <= routes[k].num_nodes_; ++i) {
                    reinsertion_cost = Instance::instance()->EdgeCost(routes[k].visited_nodes_[i-1], routes[k].visited_nodes_[i+1])
                                       - Instance::instance()->EdgeCost(routes[k].visited_nodes_[i-1], routes[k].visited_nodes_[i])
                                       - Instance::instance()->EdgeCost(routes[k].visited_nodes_[i], routes[k].visited_nodes_[i+1])
                                       + Instance::instance()->EdgeCost(0, routes[k].visited_nodes_[i])
                                       + Instance::instance()->EdgeCost(routes[k].visited_nodes_[i], 0);
                    if (cmp(reinsertion_cost, best_reinsertion_cost) < 0) {
                        best_reinsertion_cost = reinsertion_cost;
                        orig_vhc = k;
                        orig_pos = i;
                    }
                }
            }
        }

        routes[current_num_routes].InsertVisitedNode(Route::first_insertable_pos_, routes[orig_vhc].visited_nodes_[orig_pos]);
        routes[orig_vhc].RemoveVisitedNodeFromPos(orig_pos);

        ++current_num_routes;
    }
}

void ConstructiveHeuristics::AlignPickupAndDelivery(Solution & solution, Routes & pickup_routes, Routes & delivery_routes) {
    int num_requests = Instance::instance()->num_requests();
    int num_vehicles = Instance::instance()->num_vehicles();

    multimap<int, pair<int, int> > diffs_pic_del;
    int in_common, diffs;
    for (int k1 = 0; k1 < num_vehicles; ++k1) {
        for (int k2 = 0; k2 < num_vehicles; ++k2) {
            in_common = 0;
            for (int i = 1; i <= pickup_routes[k1].num_nodes_; ++i) {
                for (int j = 1; j <= delivery_routes[k2].num_nodes_; ++j) {
                    if (pickup_routes[k1].visited_nodes_[i] + num_requests == delivery_routes[k2].visited_nodes_[j]) {
                        ++in_common;
                        break;
                    }
                }
            }
            diffs = (pickup_routes[k1].num_nodes_ - in_common) + (delivery_routes[k2].num_nodes_ - in_common);
            diffs_pic_del.insert(make_pair(diffs, make_pair(k1,k2)));
        }
    }

    solution.routes_pickup_ = pickup_routes;
    vector<bool> assigned_pic(num_vehicles, false), assigned_del(num_vehicles, false);
    for (auto it = diffs_pic_del.begin(); it != diffs_pic_del.end(); ++it) {
        if (!assigned_pic[it->second.first] && !assigned_del[it->second.second]) {
            solution.routes_delivery_[it->second.first] = delivery_routes[it->second.second];
            assigned_pic[it->second.first] = assigned_del[it->second.second] = true;
        }
    }
}

void ConstructiveHeuristics::DefineScheduling(Solution & solution) {
    int num_requests = Instance::instance()->num_requests();
    int num_vehicles = Instance::instance()->num_vehicles();

    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 1; i <= solution.routes_pickup_[k].num_nodes_; ++i) {
            solution.schedule_manager_.pic_del_[solution.routes_pickup_[k].visited_nodes_[i]].first = k;
        }
    }
    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 1; i <= solution.routes_delivery_[k].num_nodes_; ++i) {
            solution.schedule_manager_.pic_del_[solution.routes_delivery_[k].visited_nodes_[i] - num_requests].second = k;
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        solution.schedule_manager_.schedules_[k].start_time_ =
                solution.schedule_manager_.schedules_[k].unload_end_ =
                        solution.schedule_manager_.schedules_[k].completion_time_ =
                                solution.routes_pickup_[k].cost_;
    }

    vector<double> vhc_start_reload(num_vehicles); // after reload preparation
    vector<double> req_start_reload(num_requests + 1);
    vector<bool> vhc_do_unload(num_vehicles, false), vhc_do_reload(num_vehicles, false);

    for (int r = 1; r <= num_requests; ++r) {
        if (solution.schedule_manager_.pic_del_[r].first != solution.schedule_manager_.pic_del_[r].second) {
            vhc_do_unload[solution.schedule_manager_.pic_del_[r].first] = true;
            vhc_do_reload[solution.schedule_manager_.pic_del_[r].second] = true;
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        vhc_start_reload[k] = solution.schedule_manager_.schedules_[k].start_time_;
        if (vhc_do_unload[k]) {
            vhc_start_reload[k] += Instance::instance()->unload_preparation_time();
        }
        if (vhc_do_reload[k]) {
            vhc_start_reload[k] += Instance::instance()->reload_preparation_time();
        }
    }

    for (int i = 1; i <= num_requests; ++i) {
        if (solution.schedule_manager_.pic_del_[i].first != solution.schedule_manager_.pic_del_[i].second) {
            req_start_reload[i] = solution.schedule_manager_.schedules_[solution.schedule_manager_.pic_del_[i].first].start_time_
                                  + Instance::instance()->unload_preparation_time()
                                  + Instance::instance()->unloading_time(i)
                                  + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[i].first, solution.schedule_manager_.pic_del_[i].second);
            vhc_start_reload[solution.schedule_manager_.pic_del_[i].first] += Instance::instance()->unloading_time(i);
        }
    }

    multimap<double, int> lateness;
    vector<bool> scheduled(num_requests + 1, false);
    for (int r = 1; r <= num_requests; ++r) {
        if (solution.schedule_manager_.pic_del_[r].first != solution.schedule_manager_.pic_del_[r].second) {
            lateness.insert(make_pair(req_start_reload[r] - vhc_start_reload[solution.schedule_manager_.pic_del_[r].second], r));
        } else {
            scheduled[r] = true;
        }
    }

    int curr_req, pic_vhc, del_vhc;

    for (auto rit = lateness.rbegin(); rit != lateness.rend(); ++rit) {
        curr_req = rit->second;
        pic_vhc = solution.schedule_manager_.pic_del_[curr_req].first;
        del_vhc = solution.schedule_manager_.pic_del_[curr_req].second;

        if (solution.schedule_manager_.schedules_[pic_vhc].unloaded_reqs_.empty()) {
            solution.schedule_manager_.schedules_[pic_vhc].unloaded_reqs_.push_back(UnloadUnit(curr_req, solution.schedule_manager_.schedules_[pic_vhc].start_time_ + Instance::instance()->unload_preparation_time()));
            solution.schedule_manager_.schedules_[pic_vhc].unload_end_ += Instance::instance()->unload_preparation_time() + Instance::instance()->unloading_time(curr_req);
        } else {
            solution.schedule_manager_.schedules_[pic_vhc].unloaded_reqs_.push_back(UnloadUnit(curr_req, solution.schedule_manager_.schedules_[pic_vhc].unload_end_));
            solution.schedule_manager_.schedules_[pic_vhc].unload_end_ += Instance::instance()->unloading_time(curr_req);
        }

        solution.schedule_manager_.schedules_[del_vhc].reloaded_reqs_.push_back(ReloadUnit(curr_req, solution.schedule_manager_.schedules_[pic_vhc].unload_end_ + Instance::instance()->between_docks_time(pic_vhc, del_vhc)));
    }

    for (int k = 0; k < num_vehicles; ++k) {
        if (vhc_do_reload[k]) {
            sort(solution.schedule_manager_.schedules_[k].reloaded_reqs_.begin(), solution.schedule_manager_.schedules_[k].reloaded_reqs_.end());
            solution.schedule_manager_.schedules_[k].reloaded_reqs_[0].start_time_ = max(solution.schedule_manager_.schedules_[k].reloaded_reqs_[0].availability_time_,
                                                                                         solution.schedule_manager_.schedules_[k].unload_end_ + Instance::instance()->reload_preparation_time());
            for (uint i = 1; i < solution.schedule_manager_.schedules_[k].reloaded_reqs_.size(); ++i) {
                solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_ = max(solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].availability_time_,
                                                                                             solution.schedule_manager_.schedules_[k].reloaded_reqs_[i-1].start_time_
                                                                                             + Instance::instance()->reloading_time(solution.schedule_manager_.schedules_[k].reloaded_reqs_[i-1].ind_));
            }

            solution.schedule_manager_.schedules_[k].completion_time_ = solution.schedule_manager_.schedules_[k].reloaded_reqs_.back().start_time_
                                                                        + Instance::instance()->reloading_time(solution.schedule_manager_.schedules_[k].reloaded_reqs_.back().ind_);
        } else {
            solution.schedule_manager_.schedules_[k].completion_time_ = solution.schedule_manager_.schedules_[k].unload_end_;
        }
    }

}

void ConstructiveHeuristics::DefineScheduling2(Solution & solution) {
    int num_requests = Instance::instance()->num_requests();
    int num_vehicles = Instance::instance()->num_vehicles();

    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 1; i <= solution.routes_pickup_[k].num_nodes_; ++i) {
            solution.schedule_manager_.pic_del_[solution.routes_pickup_[k].visited_nodes_[i]].first = k;
        }
    }
    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 1; i <= solution.routes_delivery_[k].num_nodes_; ++i) {
            solution.schedule_manager_.pic_del_[solution.routes_delivery_[k].visited_nodes_[i] - num_requests].second = k;
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        solution.schedule_manager_.schedules_[k].start_time_ =
                solution.schedule_manager_.schedules_[k].unload_end_ =
                        solution.schedule_manager_.schedules_[k].completion_time_ =
                                solution.routes_pickup_[k].cost_;
    }

    vector<bool> vhc_do_unload(num_vehicles, false), vhc_do_reload(num_vehicles, false);
    vector<vector<int>> unloaded_req_per_vhc(num_vehicles);

    for (int r = 1; r <= num_requests; ++r) {
        if (solution.schedule_manager_.pic_del_[r].first != solution.schedule_manager_.pic_del_[r].second) {
            vhc_do_unload[solution.schedule_manager_.pic_del_[r].first] = true;
            vhc_do_reload[solution.schedule_manager_.pic_del_[r].second] = true;

            unloaded_req_per_vhc[solution.schedule_manager_.pic_del_[r].first].push_back(r);

            solution.schedule_manager_.schedules_[solution.schedule_manager_.pic_del_[r].first].unload_end_ += Instance::instance()->unloading_time(r);
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        if (vhc_do_unload[k]) {
            solution.schedule_manager_.schedules_[k].unload_end_ += Instance::instance()->unload_preparation_time();
        }
    }

    vector<double> vhc_start_reload(num_vehicles); // after reload preparation

    for (int k = 0; k < num_vehicles; ++k) {
        if (vhc_do_reload[k]) {
            vhc_start_reload[k] = solution.schedule_manager_.schedules_[k].unload_end_ + Instance::instance()->reload_preparation_time();
        }
    }

    map<int, map<int, double>> relative_reload_avail;
    double start_unload;

    for (int k = 0; k < num_vehicles; ++k) {
        if (vhc_do_unload[k]) {
            start_unload = solution.schedule_manager_.schedules_[k].start_time_ + Instance::instance()->unload_preparation_time();
            for (uint i = 0; i < unloaded_req_per_vhc[k].size(); ++i) {
                for (uint j = 0; j < unloaded_req_per_vhc[k].size(); ++j) {
                    if (i != j) {
                        relative_reload_avail[unloaded_req_per_vhc[k][i]][unloaded_req_per_vhc[k][j]] = start_unload
                                                                                                        + Instance::instance()->unloading_time(unloaded_req_per_vhc[k][i])
                                                                                                        + Instance::instance()->unloading_time(unloaded_req_per_vhc[k][j])
                                                                                                        + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[unloaded_req_per_vhc[k][j]].first,
                                                                                                                                                   solution.schedule_manager_.pic_del_[unloaded_req_per_vhc[k][j]].second);
                    } else {
                        relative_reload_avail[unloaded_req_per_vhc[k][i]][unloaded_req_per_vhc[k][j]] = start_unload
                                                                                                        + Instance::instance()->unloading_time(unloaded_req_per_vhc[k][i])
                                                                                                        + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[unloaded_req_per_vhc[k][i]].first,
                                                                                                                                                   solution.schedule_manager_.pic_del_[unloaded_req_per_vhc[k][i]].second);
                    }
                }
            }
        }
    }

    double current_sum, best_sum;
    int best_req_ind, best_req_unl_time;

    for (int k = 0; k < num_vehicles; ++k) {
        if (vhc_do_unload[k]) {
            start_unload = solution.schedule_manager_.schedules_[k].start_time_ + Instance::instance()->unload_preparation_time();
            while (!unloaded_req_per_vhc[k].empty()) {
                best_sum = 1000000000.0;
                best_req_ind = best_req_unl_time = -1;
                for (uint i = 0; i < unloaded_req_per_vhc[k].size(); ++i) {
                    current_sum = 0.0;
                    for (uint j = 0; j < unloaded_req_per_vhc[k].size(); ++j) {
                        current_sum += max(relative_reload_avail[unloaded_req_per_vhc[k][i]][unloaded_req_per_vhc[k][j]] + Instance::instance()->reloading_time(unloaded_req_per_vhc[k][j]),
                                           vhc_start_reload[solution.schedule_manager_.pic_del_[unloaded_req_per_vhc[k][j]].second] + Instance::instance()->reloading_time(unloaded_req_per_vhc[k][j]));
                    }

                    if (cmp(current_sum, best_sum) < 0
                        || (cmp(current_sum, best_sum) == 0 && Instance::instance()->unloading_time(unloaded_req_per_vhc[k][i]) < best_req_unl_time)) {

                        best_sum = current_sum;
                        best_req_ind = i;
                        best_req_unl_time = Instance::instance()->unloading_time(unloaded_req_per_vhc[k][i]);
                    }
                }

                solution.schedule_manager_.schedules_[k].unloaded_reqs_.push_back(UnloadUnit(unloaded_req_per_vhc[k][best_req_ind], start_unload));
                int del_vhc = solution.schedule_manager_.pic_del_[unloaded_req_per_vhc[k][best_req_ind]].second;
                solution.schedule_manager_.schedules_[del_vhc].reloaded_reqs_.push_back(ReloadUnit(unloaded_req_per_vhc[k][best_req_ind], relative_reload_avail[unloaded_req_per_vhc[k][best_req_ind]][unloaded_req_per_vhc[k][best_req_ind]]));
                start_unload += best_req_unl_time;

                unloaded_req_per_vhc[k].erase(unloaded_req_per_vhc[k].begin() + best_req_ind);

                for (uint i = 0; i < unloaded_req_per_vhc[k].size(); ++i) {
                    for (uint j = 0; j < unloaded_req_per_vhc[k].size(); ++j) {
                        relative_reload_avail[unloaded_req_per_vhc[k][i]][unloaded_req_per_vhc[k][j]] += best_req_unl_time;
                    }
                }
            }
            solution.schedule_manager_.schedules_[k].unload_end_ = start_unload;
        }
    }

    double infeas_st_av_size;
    for (int k = 0; k < num_vehicles; ++k) {
        if (vhc_do_reload[k]) {
            sort(solution.schedule_manager_.schedules_[k].reloaded_reqs_.begin(), solution.schedule_manager_.schedules_[k].reloaded_reqs_.end());
            solution.schedule_manager_.schedules_[k].reloaded_reqs_[0].start_time_ = solution.schedule_manager_.schedules_[k].unload_end_ + Instance::instance()->reload_preparation_time();

            infeas_st_av_size = solution.schedule_manager_.schedules_[k].reloaded_reqs_[0].start_time_ - solution.schedule_manager_.schedules_[k].reloaded_reqs_[0].availability_time_;
            for (uint i = 1; i < solution.schedule_manager_.schedules_[k].reloaded_reqs_.size(); ++i) {
                solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_ = solution.schedule_manager_.schedules_[k].reloaded_reqs_[i-1].start_time_
                                                                                         + Instance::instance()->reloading_time(solution.schedule_manager_.schedules_[k].reloaded_reqs_[i-1].ind_);

                if (cmp(infeas_st_av_size, solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_ - solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].availability_time_) > 0) {
                    infeas_st_av_size = solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_ - solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].availability_time_;
                }
            }

            if (cmp(infeas_st_av_size, 0.0) < 0) {
                for (uint i = 0; i < solution.schedule_manager_.schedules_[k].reloaded_reqs_.size(); ++i) {
                    solution.schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_ -= infeas_st_av_size;
                }
            }

            solution.schedule_manager_.schedules_[k].completion_time_ = solution.schedule_manager_.schedules_[k].reloaded_reqs_.back().start_time_
                                                                        + Instance::instance()->reloading_time(solution.schedule_manager_.schedules_[k].reloaded_reqs_.back().ind_);
        } else {
            solution.schedule_manager_.schedules_[k].completion_time_ = solution.schedule_manager_.schedules_[k].unload_end_;
        }
    }

}
