#include "LocalSearches.h"

bool LocalSearches::IR_1opt(Solution & solution, LS_strat search_strat) {

    int num_requests = Instance::instance()->num_requests();
    bool improve, general_improvement, is_pickup, best_is_pickup;
    int curr_pos, curr_vhc, best_orig_pos = -1, best_dest_pos = -1, best_vhc = -1, selected_req;
    double best_cost, temp_cost, current_cost;

    vector<int> selectable_nodes(2 * num_requests);
    iota(selectable_nodes.begin(), selectable_nodes.end(), 1);
    shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;
    while (improve) {
        improve = false;
        best_cost = solution.cost_;

        for (auto selected_node = selectable_nodes.begin(); selected_node != selectable_nodes.end(); ++selected_node) {

            is_pickup = *selected_node <= num_requests ? true : false;
            Routes & routes = is_pickup ? solution.routes_pickup_ : solution.routes_delivery_;
            selected_req = is_pickup ? *selected_node : *selected_node - num_requests;
            curr_vhc = is_pickup ? solution.schedule_manager_.pic_del_[selected_req].first : solution.schedule_manager_.pic_del_[selected_req].second;
            curr_pos = find(routes[curr_vhc].visited_nodes_.begin(), routes[curr_vhc].visited_nodes_.end(), *selected_node) - routes[curr_vhc].visited_nodes_.begin();

            temp_cost = solution.cost_;

            for (int j = Route::first_insertable_pos_; j <= routes[curr_vhc].last_insertable_pos_; ++j) {
                if (j != curr_pos && j != curr_pos + 1) {
                    current_cost = temp_cost + (is_pickup ? IR_1opt_pickup_delta_cost(solution, curr_vhc, curr_pos, j) : IR_1opt_delivery_delta_cost(solution, curr_vhc, curr_pos, j));

                    if (cmp(current_cost, best_cost) < 0) {
                        best_cost = current_cost;
                        best_vhc = curr_vhc;
                        best_orig_pos = curr_pos;
                        best_dest_pos = j;
                        best_is_pickup = is_pickup;
                        improve = true;
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;
                if (best_dest_pos > best_orig_pos) {
                    --best_dest_pos;
                }
                if (best_is_pickup) {
                    IR_move_1opt_pickup_route(solution, best_vhc, best_orig_pos, best_dest_pos);
                } else {
                    IR_move_1opt_delivery_route(solution, best_vhc, best_orig_pos, best_dest_pos);
                }

                if (search_strat == LS_strat::complete_first_improv_) {
                    shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);
                    break;
                } else {
                    return general_improvement;
                }
            }
        }

        if (improve && (search_strat == LS_strat::complete_best_improv_ || search_strat == LS_strat::one_best_improv_)) {
            general_improvement = true;
            if (best_dest_pos > best_orig_pos) {
                --best_dest_pos;
            }
            if (best_is_pickup) {
                IR_move_1opt_pickup_route(solution, best_vhc, best_orig_pos, best_dest_pos);
            } else {
                IR_move_1opt_delivery_route(solution, best_vhc, best_orig_pos, best_dest_pos);
            }

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }

    }

    return general_improvement;
}

double LocalSearches::IR_1opt_pickup_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos) {

    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    double vhc_route_cost;
    double vhc_unload_end;

    vhc_route_cost = solution.routes_pickup_[route].cost_
                     + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos - 1], solution.routes_pickup_[route].visited_nodes_[orig_pos + 1])
                     - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos - 1], solution.routes_pickup_[route].visited_nodes_[orig_pos])
                     - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos], solution.routes_pickup_[route].visited_nodes_[orig_pos + 1])
                     - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos - 1], solution.routes_pickup_[route].visited_nodes_[dest_pos])
                     + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos - 1], solution.routes_pickup_[route].visited_nodes_[orig_pos])
                     + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos], solution.routes_pickup_[route].visited_nodes_[dest_pos]);

    if (cmp(vhc_route_cost, solution.routes_pickup_[route].cost_) >= 0) {
        return vhc_route_cost - solution.routes_pickup_[route].cost_;
    }

    // unloading updates
    double aux_start_unload;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    // on vhc_1
    aux_start_unload = vhc_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[route].unloaded_reqs_.size(); ++i) {
        req_start_unload[schedules[route].unloaded_reqs_[i].ind_] = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[route].unloaded_reqs_[i].ind_);
    }

    if (schedules[route].unloaded_reqs_.empty()) {
        vhc_unload_end = vhc_route_cost;
    } else {
        vhc_unload_end = aux_start_unload;
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(route);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_delta, curr_unload_end, infeas_st_av_size;

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {

        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, route);
            }
        }

        if (*rld_vhc == route) {
            curr_unload_end = vhc_unload_end;
        } else {
            curr_unload_end = schedules[*rld_vhc].unload_end_;
        }

        if (!rs.empty()) {
            sort(rs.begin(), rs.end());
            rs[0].start_time_ = curr_unload_end + Instance::instance()->reload_preparation_time();
            infeas_st_av_size = rs[0].start_time_ - rs[0].availability_time_;
            for (uint i = 1; i < rs.size(); ++i) {
                rs[i].start_time_ = rs[i-1].start_time_ + Instance::instance()->reloading_time(rs[i-1].ind_);
                if (cmp(infeas_st_av_size, rs[i].start_time_ - rs[i].availability_time_) > 0) {
                    infeas_st_av_size = rs[i].start_time_ - rs[i].availability_time_;
                }
            }

            if (cmp(infeas_st_av_size, 0.0) < 0) {
                for (uint i = 0; i < rs.size(); ++i) {
                    rs[i].start_time_ -= infeas_st_av_size;
                }
            }

            curr_delta = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
            delta_cost += curr_delta;
        } else {
            curr_delta = curr_unload_end - schedules[*rld_vhc].completion_time_;
            delta_cost += curr_delta;
        }
    }

    return delta_cost;
}

double LocalSearches::IR_1opt_delivery_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos) {

    double delta_cost = 0.0;
    double vhc_route_cost;

    vhc_route_cost = solution.routes_delivery_[route].cost_
                     + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos - 1], solution.routes_delivery_[route].visited_nodes_[orig_pos + 1])
                     - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos - 1], solution.routes_delivery_[route].visited_nodes_[orig_pos])
                     - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos], solution.routes_delivery_[route].visited_nodes_[orig_pos + 1])
                     - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos - 1], solution.routes_delivery_[route].visited_nodes_[dest_pos])
                     + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos - 1], solution.routes_delivery_[route].visited_nodes_[orig_pos])
                     + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos], solution.routes_delivery_[route].visited_nodes_[dest_pos]);

    delta_cost = vhc_route_cost - solution.routes_delivery_[route].cost_;

    return delta_cost;
}

void LocalSearches::IR_move_1opt_pickup_route(Solution & solution, const int route, const int orig_pos, const int dest_pos) {
    int node = solution.routes_pickup_[route].visited_nodes_[orig_pos];
    double old_cost = solution.routes_pickup_[route].cost_;

    solution.routes_pickup_[route].RemoveVisitedNodeFromPos(orig_pos);
    solution.routes_pickup_[route].InsertVisitedNode(dest_pos, node);

    double diff_cost = old_cost - solution.routes_pickup_[route].cost_;
    double completion_diffs = 0.0;

    IR_update_schedule(solution, route, diff_cost, completion_diffs);

    solution.cost_ -= completion_diffs;
}

void LocalSearches::IR_move_1opt_delivery_route(Solution & solution, const int route, const int orig_pos, const int dest_pos) {
    int node = solution.routes_delivery_[route].visited_nodes_[orig_pos];
    double old_cost = solution.routes_delivery_[route].cost_;

    solution.routes_delivery_[route].RemoveVisitedNodeFromPos(orig_pos);
    solution.routes_delivery_[route].InsertVisitedNode(dest_pos, node);

    solution.cost_ -= old_cost - solution.routes_delivery_[route].cost_;
}

void LocalSearches::IR_update_schedule(Solution & solution, int route, double diff_cost, double & completion_diffs) {
    // update unload times and reload availability times
    int dest_vhc, ind;
    set<int> vhc_to_update_reload;
    Schedules & schedules = solution.schedule_manager_.schedules_;

    schedules[route].start_time_ -= diff_cost;
    vhc_to_update_reload.insert(route);

    for (uint i = 0; i < schedules[route].unloaded_reqs_.size(); ++i) {
        schedules[route].unloaded_reqs_[i].start_time_ -= diff_cost;
        ind = schedules[route].unloaded_reqs_[i].ind_;
        dest_vhc = solution.schedule_manager_.pic_del_[ind].second;

        for (uint j = 0; j < schedules[dest_vhc].reloaded_reqs_.size(); ++j) {
            if (schedules[dest_vhc].reloaded_reqs_[j].ind_ == ind) {
                schedules[dest_vhc].reloaded_reqs_[j].availability_time_ -= diff_cost;
                vhc_to_update_reload.insert(dest_vhc);
                break;
            }
        }
    }

    schedules[route].unload_end_ -= diff_cost;

    double infeas_st_av_size, completion_before;
    for (auto it = vhc_to_update_reload.begin(); it != vhc_to_update_reload.end(); ++it) {
        if (!schedules[*it].reloaded_reqs_.empty()) {
            sort(schedules[*it].reloaded_reqs_.begin(), schedules[*it].reloaded_reqs_.end());
            schedules[*it].reloaded_reqs_[0].start_time_ = schedules[*it].unload_end_ + Instance::instance()->reload_preparation_time();

            infeas_st_av_size = schedules[*it].reloaded_reqs_[0].start_time_ - schedules[*it].reloaded_reqs_[0].availability_time_;
            for (uint i = 1; i < schedules[*it].reloaded_reqs_.size(); ++i) {
                schedules[*it].reloaded_reqs_[i].start_time_ = schedules[*it].reloaded_reqs_[i-1].start_time_
                                                               + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_[i-1].ind_);

                if (cmp(infeas_st_av_size, schedules[*it].reloaded_reqs_[i].start_time_ - schedules[*it].reloaded_reqs_[i].availability_time_) > 0) {
                    infeas_st_av_size = schedules[*it].reloaded_reqs_[i].start_time_ - schedules[*it].reloaded_reqs_[i].availability_time_;
                }
            }

            if (cmp(infeas_st_av_size, 0.0) < 0) {
                for (uint i = 0; i < schedules[*it].reloaded_reqs_.size(); ++i) {
                    schedules[*it].reloaded_reqs_[i].start_time_ -= infeas_st_av_size;
                }
            }

            completion_before = schedules[*it].completion_time_;
            schedules[*it].completion_time_ = schedules[*it].reloaded_reqs_.back().start_time_
                                              + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_.back().ind_);
            completion_diffs += completion_before - schedules[*it].completion_time_;
        } else {
            completion_before = schedules[*it].completion_time_;
            schedules[*it].completion_time_ = schedules[*it].unload_end_;
            completion_diffs += completion_before - schedules[*it].completion_time_;
        }
    }
}

bool LocalSearches::IR_2opt(Solution & solution, LS_strat search_strat) {

    int num_requests = Instance::instance()->num_requests();
    bool improve, general_improvement, is_pickup, best_is_pickup;
    int curr_pos, curr_vhc, best_first_pos = -1, best_last_pos = -1, best_vhc = -1, selected_req;
    double best_cost, temp_cost, current_cost;
    map<int, set<int>> edges_already_used;
    int curr_edge, j_edge, num_total_nodes = 2 * num_requests + 1;

    vector<int> selectable_nodes(2 * num_requests);
    iota(selectable_nodes.begin(), selectable_nodes.end(), 1);
    shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;
    while (improve) {
        improve = false;
        best_cost = solution.cost_;
        edges_already_used.clear();

        for (auto selected_node = selectable_nodes.begin(); selected_node != selectable_nodes.end(); ++selected_node) {

            is_pickup = *selected_node <= num_requests ? true : false;
            Routes & routes = is_pickup ? solution.routes_pickup_ : solution.routes_delivery_;
            selected_req = is_pickup ? *selected_node : *selected_node - num_requests;
            curr_vhc = is_pickup ? solution.schedule_manager_.pic_del_[selected_req].first : solution.schedule_manager_.pic_del_[selected_req].second;
            curr_pos = find(routes[curr_vhc].visited_nodes_.begin(), routes[curr_vhc].visited_nodes_.end(), *selected_node) - routes[curr_vhc].visited_nodes_.begin();

            temp_cost = solution.cost_;

            for (int j = Route::first_insertable_pos_; j < routes[curr_vhc].last_insertable_pos_; ++j) {
                if (j < curr_pos) {
                    curr_edge = routes[curr_vhc].visited_nodes_[curr_pos] * num_total_nodes + routes[curr_vhc].visited_nodes_[curr_pos+1];
                    j_edge = routes[curr_vhc].visited_nodes_[j-1] * num_total_nodes + routes[curr_vhc].visited_nodes_[j];
                    if (edges_already_used[curr_edge].find(j_edge) == edges_already_used[curr_edge].end()) {

                        edges_already_used[j_edge].insert(curr_edge);

                        current_cost = temp_cost + (is_pickup ? IR_2opt_pickup_delta_cost(solution, curr_vhc, curr_pos, j) : IR_2opt_delivery_delta_cost(solution, curr_vhc, curr_pos, j));

                        if (cmp(current_cost, best_cost) < 0) {
                            best_cost = current_cost;
                            if (j < curr_pos) {
                                best_first_pos = j;
                                best_last_pos = curr_pos;
                            } else {
                                best_first_pos = curr_pos;
                                best_last_pos = j;
                            }
                            best_vhc = curr_vhc;
                            best_is_pickup = is_pickup;
                            improve = true;
                        }
                    }
                } else if (j > curr_pos) {
                    curr_edge = routes[curr_vhc].visited_nodes_[curr_pos-1] * num_total_nodes + routes[curr_vhc].visited_nodes_[curr_pos];
                    j_edge = routes[curr_vhc].visited_nodes_[j] * num_total_nodes + routes[curr_vhc].visited_nodes_[j+1];
                    if (edges_already_used[curr_edge].find(j_edge) == edges_already_used[curr_edge].end()) {

                        edges_already_used[j_edge].insert(curr_edge);

                        current_cost = temp_cost + (is_pickup ? IR_2opt_pickup_delta_cost(solution, curr_vhc, curr_pos, j) : IR_2opt_delivery_delta_cost(solution, curr_vhc, curr_pos, j));

                        if (cmp(current_cost, best_cost) < 0) {
                            best_cost = current_cost;
                            if (j < curr_pos) {
                                best_first_pos = j;
                                best_last_pos = curr_pos;
                            } else {
                                best_first_pos = curr_pos;
                                best_last_pos = j;
                            }
                            best_vhc = curr_vhc;
                            best_is_pickup = is_pickup;
                            improve = true;
                        }
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;
                if (best_is_pickup) {
                    IR_move_2opt_pickup_route(solution, best_vhc, best_first_pos, best_last_pos);
                } else {
                    IR_move_2opt_delivery_route(solution, best_vhc, best_first_pos, best_last_pos);
                }

                if (search_strat == LS_strat::complete_first_improv_) {
                    shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);
                    break;
                } else {
                    return general_improvement;
                }
            }
        }

        if (improve && (search_strat == LS_strat::complete_best_improv_ || search_strat == LS_strat::one_best_improv_)) {
            general_improvement = true;
            if (best_is_pickup) {
                IR_move_2opt_pickup_route(solution, best_vhc, best_first_pos, best_last_pos);
            } else {
                IR_move_2opt_delivery_route(solution, best_vhc, best_first_pos, best_last_pos);
            }

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }

    }

    return general_improvement;
}

double LocalSearches::IR_2opt_pickup_delta_cost(Solution & solution, const int route, const int first_pos, const int last_pos) {

    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    double vhc_route_cost;
    double vhc_unload_end;
    int pos_1, pos_2;

    if (first_pos < last_pos) {
        pos_1 = first_pos;
        pos_2 = last_pos;
    } else {
        pos_1 = last_pos;
        pos_2 = first_pos;
    }

    vhc_route_cost = solution.routes_pickup_[route].cost_
                     - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[pos_2], solution.routes_pickup_[route].visited_nodes_[pos_2 + 1])
                     - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[pos_1 - 1], solution.routes_pickup_[route].visited_nodes_[pos_1])
                     + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[pos_1 - 1], solution.routes_pickup_[route].visited_nodes_[pos_2])
                     + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[pos_1], solution.routes_pickup_[route].visited_nodes_[pos_2 + 1]);

    if (cmp(vhc_route_cost, solution.routes_pickup_[route].cost_) >= 0) {
        return vhc_route_cost - solution.routes_pickup_[route].cost_;
    }

    // unloading updates
    double aux_start_unload;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    // on vhc_1
    aux_start_unload = vhc_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[route].unloaded_reqs_.size(); ++i) {
        req_start_unload[schedules[route].unloaded_reqs_[i].ind_] = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[route].unloaded_reqs_[i].ind_);
    }

    if (schedules[route].unloaded_reqs_.empty()) {
        vhc_unload_end = vhc_route_cost;
    } else {
        vhc_unload_end = aux_start_unload;
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(route);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_delta, curr_unload_end, infeas_st_av_size;

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {

        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, route);
            }
        }

        if (*rld_vhc == route) {
            curr_unload_end = vhc_unload_end;
        } else {
            curr_unload_end = schedules[*rld_vhc].unload_end_;
        }

        if (!rs.empty()) {
            sort(rs.begin(), rs.end());
            rs[0].start_time_ = curr_unload_end + Instance::instance()->reload_preparation_time();
            infeas_st_av_size = rs[0].start_time_ - rs[0].availability_time_;
            for (uint i = 1; i < rs.size(); ++i) {
                rs[i].start_time_ = rs[i-1].start_time_ + Instance::instance()->reloading_time(rs[i-1].ind_);
                if (cmp(infeas_st_av_size, rs[i].start_time_ - rs[i].availability_time_) > 0) {
                    infeas_st_av_size = rs[i].start_time_ - rs[i].availability_time_;
                }
            }

            if (cmp(infeas_st_av_size, 0.0) < 0) {
                for (uint i = 0; i < rs.size(); ++i) {
                    rs[i].start_time_ -= infeas_st_av_size;
                }
            }

            curr_delta = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
            delta_cost += curr_delta;
        } else {
            curr_delta = curr_unload_end - schedules[*rld_vhc].completion_time_;
            delta_cost += curr_delta;
        }
    }

    return delta_cost;
}

double LocalSearches::IR_2opt_delivery_delta_cost(Solution & solution, const int route, const int first_pos, const int last_pos) {

    double delta_cost = 0.0;
    double vhc_route_cost;
    int pos_1, pos_2;

    if (first_pos < last_pos) {
        pos_1 = first_pos;
        pos_2 = last_pos;
    } else {
        pos_1 = last_pos;
        pos_2 = first_pos;
    }

    vhc_route_cost = solution.routes_delivery_[route].cost_
                     - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[pos_2], solution.routes_delivery_[route].visited_nodes_[pos_2 + 1])
                     - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[pos_1 - 1], solution.routes_delivery_[route].visited_nodes_[pos_1])
                     + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[pos_1 - 1], solution.routes_delivery_[route].visited_nodes_[pos_2])
                     + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[pos_1], solution.routes_delivery_[route].visited_nodes_[pos_2 + 1]);

    delta_cost = vhc_route_cost - solution.routes_delivery_[route].cost_;

    return delta_cost;
}

void LocalSearches::IR_move_2opt_pickup_route(Solution & solution, const int route, const int first_pos, const int last_pos) {
    double old_cost = solution.routes_pickup_[route].cost_;

    solution.routes_pickup_[route].ReversePartVisitedNodes(first_pos, last_pos);

    double diff_cost = old_cost - solution.routes_pickup_[route].cost_;
    double completion_diffs = 0.0;

    IR_update_schedule(solution, route, diff_cost, completion_diffs);

    solution.cost_ -= completion_diffs;
}

void LocalSearches::IR_move_2opt_delivery_route(Solution & solution, const int route, const int first_pos, const int last_pos) {
    double old_cost = solution.routes_delivery_[route].cost_;

    solution.routes_delivery_[route].ReversePartVisitedNodes(first_pos, last_pos);

    solution.cost_ -= old_cost - solution.routes_delivery_[route].cost_;
}

bool LocalSearches::IR_Exchange(Solution & solution, LS_strat search_strat) {

    int num_requests = Instance::instance()->num_requests();
    bool improve, general_improvement, is_pickup, best_is_pickup;
    int curr_pos, curr_vhc, best_pos_1 = -1, best_pos_2 = -1, best_vhc = -1, selected_req;
    double best_cost, temp_cost, current_cost;
    vector<bool> nodes_already_selected(2 * num_requests + 1);

    vector<int> selectable_nodes(2 * num_requests);
    iota(selectable_nodes.begin(), selectable_nodes.end(), 1);
    shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;
    while (improve) {
        improve = false;
        best_cost = solution.cost_;
        nodes_already_selected.assign(2 * num_requests + 1, false);

        for (auto selected_node = selectable_nodes.begin(); selected_node != selectable_nodes.end(); ++selected_node) {

            nodes_already_selected[*selected_node] = true;
            is_pickup = *selected_node <= num_requests ? true : false;
            Routes & routes = is_pickup ? solution.routes_pickup_ : solution.routes_delivery_;
            selected_req = is_pickup ? *selected_node : *selected_node - num_requests;
            curr_vhc = is_pickup ? solution.schedule_manager_.pic_del_[selected_req].first : solution.schedule_manager_.pic_del_[selected_req].second;
            curr_pos = find(routes[curr_vhc].visited_nodes_.begin(), routes[curr_vhc].visited_nodes_.end(), *selected_node) - routes[curr_vhc].visited_nodes_.begin();

            temp_cost = solution.cost_;

            for (int j = Route::first_insertable_pos_; j < routes[curr_vhc].last_insertable_pos_; ++j) {
                if (!nodes_already_selected[routes[curr_vhc].visited_nodes_[j]] && j != curr_pos) {
                    current_cost = temp_cost + (is_pickup ? IR_exchange_pickup_delta_cost(solution, curr_vhc, curr_pos, j) : IR_exchange_delivery_delta_cost(solution, curr_vhc, curr_pos, j));
                    if (cmp(current_cost, best_cost) < 0) {
                        best_cost = current_cost;
                        best_vhc = curr_vhc;
                        best_pos_1 = curr_pos;
                        best_pos_2 = j;
                        best_is_pickup = is_pickup;
                        improve = true;
                    }
                }

            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;
                if (best_is_pickup) {
                    IR_move_exchange_pickup_route(solution, best_vhc, best_pos_1, best_pos_2);
                } else {
                    IR_move_exchange_delivery_route(solution, best_vhc, best_pos_1, best_pos_2);
                }
                if (search_strat == LS_strat::complete_first_improv_) {
                    shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);
                    break;
                } else {
                    return general_improvement;
                }
            }
        }

        if (improve && (search_strat == LS_strat::complete_best_improv_ || search_strat == LS_strat::one_best_improv_)) {
            general_improvement = true;
            if (best_is_pickup) {
                IR_move_exchange_pickup_route(solution, best_vhc, best_pos_1, best_pos_2);
            } else {
                IR_move_exchange_delivery_route(solution, best_vhc, best_pos_1, best_pos_2);
            }

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }
    }
    return general_improvement;
}

double LocalSearches::IR_exchange_pickup_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos) {

    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    double vhc_route_cost;
    double vhc_unload_end;

    if (dest_pos == orig_pos - 1) {
        vhc_route_cost = solution.routes_pickup_[route].cost_
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos - 1], solution.routes_pickup_[route].visited_nodes_[dest_pos])
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos], solution.routes_pickup_[route].visited_nodes_[orig_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos - 1], solution.routes_pickup_[route].visited_nodes_[orig_pos])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos], solution.routes_pickup_[route].visited_nodes_[orig_pos + 1]);
    } else if (dest_pos == orig_pos + 1) {
        vhc_route_cost = solution.routes_pickup_[route].cost_
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos], solution.routes_pickup_[route].visited_nodes_[dest_pos + 1])
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos - 1], solution.routes_pickup_[route].visited_nodes_[orig_pos])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos], solution.routes_pickup_[route].visited_nodes_[dest_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos - 1], solution.routes_pickup_[route].visited_nodes_[dest_pos]);
    } else {
        vhc_route_cost = solution.routes_pickup_[route].cost_
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos - 1], solution.routes_pickup_[route].visited_nodes_[dest_pos])
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos], solution.routes_pickup_[route].visited_nodes_[dest_pos + 1])
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos - 1], solution.routes_pickup_[route].visited_nodes_[orig_pos])
                         - Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos], solution.routes_pickup_[route].visited_nodes_[orig_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos - 1], solution.routes_pickup_[route].visited_nodes_[orig_pos])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos], solution.routes_pickup_[route].visited_nodes_[dest_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[orig_pos - 1], solution.routes_pickup_[route].visited_nodes_[dest_pos])
                         + Instance::instance()->EdgeCost(solution.routes_pickup_[route].visited_nodes_[dest_pos], solution.routes_pickup_[route].visited_nodes_[orig_pos + 1]);
    }

    if (cmp(vhc_route_cost, solution.routes_pickup_[route].cost_) >= 0) {
        return vhc_route_cost - solution.routes_pickup_[route].cost_;
    }

    // unloading updates
    double aux_start_unload;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    // on vhc_1
    aux_start_unload = vhc_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[route].unloaded_reqs_.size(); ++i) {
        req_start_unload[schedules[route].unloaded_reqs_[i].ind_] = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[route].unloaded_reqs_[i].ind_);
    }

    if (schedules[route].unloaded_reqs_.empty()) {
        vhc_unload_end = vhc_route_cost;
    } else {
        vhc_unload_end = aux_start_unload;
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(route);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_delta, curr_unload_end, infeas_st_av_size;

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {

        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, route);
            }
        }

        if (*rld_vhc == route) {
            curr_unload_end = vhc_unload_end;
        } else {
            curr_unload_end = schedules[*rld_vhc].unload_end_;
        }

        if (!rs.empty()) {
            sort(rs.begin(), rs.end());
            rs[0].start_time_ = curr_unload_end + Instance::instance()->reload_preparation_time();
            infeas_st_av_size = rs[0].start_time_ - rs[0].availability_time_;
            for (uint i = 1; i < rs.size(); ++i) {
                rs[i].start_time_ = rs[i-1].start_time_ + Instance::instance()->reloading_time(rs[i-1].ind_);
                if (cmp(infeas_st_av_size, rs[i].start_time_ - rs[i].availability_time_) > 0) {
                    infeas_st_av_size = rs[i].start_time_ - rs[i].availability_time_;
                }
            }

            if (cmp(infeas_st_av_size, 0.0) < 0) {
                for (uint i = 0; i < rs.size(); ++i) {
                    rs[i].start_time_ -= infeas_st_av_size;
                }
            }

            curr_delta = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
            delta_cost += curr_delta;
        } else {
            curr_delta = curr_unload_end - schedules[*rld_vhc].completion_time_;
            delta_cost += curr_delta;
        }
    }

    return delta_cost;

}

double LocalSearches::IR_exchange_delivery_delta_cost(Solution & solution, const int route, const int orig_pos, const int dest_pos) {
    double delta_cost = 0.0;
    double vhc_route_cost;

    if (dest_pos == orig_pos - 1) {
        vhc_route_cost = solution.routes_delivery_[route].cost_
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos - 1], solution.routes_delivery_[route].visited_nodes_[dest_pos])
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos], solution.routes_delivery_[route].visited_nodes_[orig_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos - 1], solution.routes_delivery_[route].visited_nodes_[orig_pos])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos], solution.routes_delivery_[route].visited_nodes_[orig_pos + 1]);
    } else if (dest_pos == orig_pos + 1) {
        vhc_route_cost = solution.routes_delivery_[route].cost_
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos], solution.routes_delivery_[route].visited_nodes_[dest_pos + 1])
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos - 1], solution.routes_delivery_[route].visited_nodes_[orig_pos])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos], solution.routes_delivery_[route].visited_nodes_[dest_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos - 1], solution.routes_delivery_[route].visited_nodes_[dest_pos]);
    } else {
        vhc_route_cost = solution.routes_delivery_[route].cost_
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos - 1], solution.routes_delivery_[route].visited_nodes_[dest_pos])
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos], solution.routes_delivery_[route].visited_nodes_[dest_pos + 1])
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos - 1], solution.routes_delivery_[route].visited_nodes_[orig_pos])
                         - Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos], solution.routes_delivery_[route].visited_nodes_[orig_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos - 1], solution.routes_delivery_[route].visited_nodes_[orig_pos])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos], solution.routes_delivery_[route].visited_nodes_[dest_pos + 1])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[orig_pos - 1], solution.routes_delivery_[route].visited_nodes_[dest_pos])
                         + Instance::instance()->EdgeCost(solution.routes_delivery_[route].visited_nodes_[dest_pos], solution.routes_delivery_[route].visited_nodes_[orig_pos + 1]);
    }

    delta_cost = vhc_route_cost - solution.routes_delivery_[route].cost_;

    return delta_cost;
}

void LocalSearches::IR_move_exchange_pickup_route(Solution & solution, const int route, const int orig_pos, const int dest_pos) {
    double old_cost = solution.routes_pickup_[route].cost_;

    solution.routes_pickup_[route].SwapVisitedNodes(orig_pos, dest_pos);

    double diff_cost = old_cost - solution.routes_pickup_[route].cost_;
    double completion_diffs = 0.0;

    IR_update_schedule(solution, route, diff_cost, completion_diffs);

    solution.cost_ -= completion_diffs;
}

void LocalSearches::IR_move_exchange_delivery_route(Solution & solution, const int route, const int orig_pos, const int dest_pos) {
    double old_cost = solution.routes_delivery_[route].cost_;

    solution.routes_delivery_[route].SwapVisitedNodes(orig_pos, dest_pos);

    solution.cost_ -= old_cost - solution.routes_delivery_[route].cost_;
}
