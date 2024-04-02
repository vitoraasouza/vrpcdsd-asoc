#include "LocalSearches.h"

bool LocalSearches::ER_1opt(Solution & solution, LS_strat search_strat) {

    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    bool improve, general_improvement, is_pickup, best_is_pickup;
    int curr_pos, curr_vhc, unl_pos, best_unl_pos = -1, best_orig_pos = -1, best_dest_pos = -1, best_orig_vhc = -1, best_dest_vhc = -1, selected_req, orig_demand, best_curr_insert_pos;
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

            if (routes[curr_vhc].num_nodes_ > 1) {
                curr_pos = find(routes[curr_vhc].visited_nodes_.begin(), routes[curr_vhc].visited_nodes_.end(), *selected_node) - routes[curr_vhc].visited_nodes_.begin();
                temp_cost = solution.cost_;
                orig_demand = Instance::instance()->node(*selected_node).demand();

                for (int k2 = 0; k2 < num_vehicles; ++k2) {
                    if (curr_vhc != k2) {
                        if (vhc_capacity - routes[k2].load_ >= orig_demand) {
                            best_curr_insert_pos = ER_1opt_best_insertion_pos(routes[k2].visited_nodes_, *selected_node);

                            current_cost = temp_cost + (is_pickup ? ER_1opt_pickup_delta_cost(solution, curr_vhc, curr_pos, k2, best_curr_insert_pos, unl_pos)
                                                        : ER_1opt_delivery_delta_cost(solution, curr_vhc, curr_pos, k2, best_curr_insert_pos, unl_pos));

                            if (cmp(current_cost, best_cost) < 0) {
                                best_cost = current_cost;
                                best_orig_pos = curr_pos;
                                best_dest_pos = best_curr_insert_pos;
                                best_orig_vhc = curr_vhc;
                                best_dest_vhc = k2;
                                best_unl_pos = unl_pos;
                                best_is_pickup = is_pickup;
                                improve = true;
                            }
                        }
                    }
                }

                if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                    general_improvement = true;
                    if (best_is_pickup) {
                        ER_move_1opt_pickup_routes(solution, best_orig_vhc, best_orig_pos, best_dest_vhc, best_dest_pos, best_unl_pos);
                    } else {
                        ER_move_1opt_delivery_routes(solution, best_orig_vhc, best_orig_pos, best_dest_vhc, best_dest_pos, best_unl_pos);
                    }

                    if (search_strat == LS_strat::complete_first_improv_) {
                        shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);
                        break;
                    } else {
                        return general_improvement;
                    }
                }
            }


        }

        if (improve && (search_strat == LS_strat::complete_best_improv_ || search_strat == LS_strat::one_best_improv_)) {
            general_improvement = true;
            if (best_is_pickup) {
                ER_move_1opt_pickup_routes(solution, best_orig_vhc, best_orig_pos, best_dest_vhc, best_dest_pos, best_unl_pos);
            } else {
                ER_move_1opt_delivery_routes(solution, best_orig_vhc, best_orig_pos, best_dest_vhc, best_dest_pos, best_unl_pos);
            }

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }

    }

    return general_improvement;
}

int LocalSearches::ER_1opt_best_insertion_pos(VisitedNodes & visited_nodes, const int insert_node) {
    double min_cost = Instance::instance()->EdgeCost(visited_nodes[0], insert_node)
                      + Instance::instance()->EdgeCost(insert_node, visited_nodes[1])
                      - Instance::instance()->EdgeCost(visited_nodes[0], visited_nodes[1]);
    int best_pos = 1;
    double current_cost;
    for (uint i = 2; i < visited_nodes.size(); ++i) {
        current_cost = Instance::instance()->EdgeCost(visited_nodes[i-1], insert_node)
                       + Instance::instance()->EdgeCost(insert_node, visited_nodes[i])
                       - Instance::instance()->EdgeCost(visited_nodes[i-1], visited_nodes[i]);
        if (cmp(current_cost, min_cost) < 0) {
            best_pos = i;
            min_cost = current_cost;
        }
    }
    return best_pos;
}

double LocalSearches::ER_1opt_pickup_delta_cost(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, int & unl_pos) {
    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    int orig_node = solution.routes_pickup_[orig_vhc].visited_nodes_[orig_pos];
    double orig_vhc_route_cost, dest_vhc_route_cost;
    double orig_vhc_unload_end, dest_vhc_unload_end;
    vector<double> start_reload(num_reqs + 1, -1.0), ct_delta(num_vhcs, 0.0);
    unl_pos = -1;

    // delta cost on routes
    orig_vhc_route_cost = solution.routes_pickup_[orig_vhc].cost_
                          + Instance::instance()->EdgeCost(solution.routes_pickup_[orig_vhc].visited_nodes_[orig_pos - 1], solution.routes_pickup_[orig_vhc].visited_nodes_[orig_pos + 1])
                          - Instance::instance()->EdgeCost(solution.routes_pickup_[orig_vhc].visited_nodes_[orig_pos - 1], orig_node)
                          - Instance::instance()->EdgeCost(orig_node, solution.routes_pickup_[orig_vhc].visited_nodes_[orig_pos + 1]);

    dest_vhc_route_cost = solution.routes_pickup_[dest_vhc].cost_
                          + Instance::instance()->EdgeCost(solution.routes_pickup_[dest_vhc].visited_nodes_[dest_pos - 1], orig_node)
                          + Instance::instance()->EdgeCost(orig_node, solution.routes_pickup_[dest_vhc].visited_nodes_[dest_pos])
                          - Instance::instance()->EdgeCost(solution.routes_pickup_[dest_vhc].visited_nodes_[dest_pos - 1], solution.routes_pickup_[dest_vhc].visited_nodes_[dest_pos]);

    // unloading updates
    double aux_start_unload;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    // on orig_vhc
    aux_start_unload = orig_vhc_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[orig_vhc].unloaded_reqs_.size(); ++i) {
        if (schedules[orig_vhc].unloaded_reqs_[i].ind_ != orig_node) {
            req_start_unload[schedules[orig_vhc].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[orig_vhc].unloaded_reqs_[i].ind_);
        }
    }

    if (schedules[orig_vhc].unloaded_reqs_.empty()
        || (schedules[orig_vhc].unloaded_reqs_.size() == 1
            && schedules[orig_vhc].unloaded_reqs_[0].ind_ ==  orig_node)) {
        orig_vhc_unload_end = orig_vhc_route_cost;
    } else {
        orig_vhc_unload_end = aux_start_unload;
    }

    // on dest_vhc
    aux_start_unload = dest_vhc_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[dest_vhc].unloaded_reqs_.size(); ++i) {
        req_start_unload[schedules[dest_vhc].unloaded_reqs_[i].ind_] = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_);
    }

    if (solution.schedule_manager_.pic_del_[orig_node].second != dest_vhc) {
        req_start_unload[orig_node] = aux_start_unload;
        dest_vhc_unload_end = aux_start_unload + Instance::instance()->unloading_time(orig_node);
    } else {
        req_start_unload[orig_node] = 0.0;
        if (schedules[dest_vhc].unloaded_reqs_.empty()) {
            dest_vhc_unload_end = dest_vhc_route_cost;
        } else {
            dest_vhc_unload_end = aux_start_unload;
        }
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(orig_vhc);
    impacted_reloads.insert(dest_vhc);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_delta, curr_unload_end, infeas_st_av_size;

    for (int k = 0; k < num_vhcs; ++k) {
        for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
            start_reload[schedules[k].reloaded_reqs_[i].ind_] = schedules[k].reloaded_reqs_[i].start_time_;
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (rs[i].ind_ != orig_node) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                } else {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == dest_vhc) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(dest_vhc, *rld_vhc);
                    }
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == dest_vhc) {
            curr_unload_end = dest_vhc_unload_end;
        } else if (*rld_vhc == orig_vhc) {
            curr_unload_end = orig_vhc_unload_end;
            if (solution.schedule_manager_.pic_del_[orig_node].first == solution.schedule_manager_.pic_del_[orig_node].second) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = orig_node;
                rs[rs.size() - 1].availability_time_ = req_start_unload[orig_node] + Instance::instance()->unloading_time(orig_node) + Instance::instance()->between_docks_time(dest_vhc, *rld_vhc);
            }
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
                    start_reload[rs[i].ind_] = rs[i].start_time_;
                }
            }

            curr_delta = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
            ct_delta[*rld_vhc] = curr_delta;
        } else {
            curr_delta = curr_unload_end - schedules[*rld_vhc].completion_time_;
            ct_delta[*rld_vhc] = curr_delta;
        }
    }

    impacted_reloads.clear();

    if (solution.schedule_manager_.pic_del_[orig_node].second != dest_vhc) {
        unl_pos = schedules[dest_vhc].unloaded_reqs_.size();
        for (int i = schedules[dest_vhc].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (cmp(Instance::instance()->unloading_time(orig_node),
                    start_reload[schedules[dest_vhc].unloaded_reqs_[i].ind_]
                    - (req_start_unload[schedules[dest_vhc].unloaded_reqs_[i].ind_]
                       + Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_)
                       + Instance::instance()->between_docks_time(dest_vhc, solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second))) <= 0) {

                req_start_unload[orig_node] -= Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[dest_vhc].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(orig_node);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[orig_node].second);
                unl_pos = i;
            } else {
                break;
            }
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (rs[i].ind_ != orig_node) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                } else {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == dest_vhc) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(dest_vhc, *rld_vhc);
                    }
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == dest_vhc) {
            curr_unload_end = dest_vhc_unload_end;
        } else if (*rld_vhc == orig_vhc) {
            curr_unload_end = orig_vhc_unload_end;
            if (solution.schedule_manager_.pic_del_[orig_node].first == solution.schedule_manager_.pic_del_[orig_node].second) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = orig_node;
                rs[rs.size() - 1].availability_time_ = req_start_unload[orig_node] + Instance::instance()->unloading_time(orig_node) + Instance::instance()->between_docks_time(dest_vhc, *rld_vhc);
            }
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
            ct_delta[*rld_vhc] = curr_delta;
        } else {
            curr_delta = curr_unload_end - schedules[*rld_vhc].completion_time_;
            ct_delta[*rld_vhc] = curr_delta;
        }
    }

    for (int k = 0; k < num_vhcs; ++k) {
        delta_cost += ct_delta[k];
    }

    return delta_cost;
}

double LocalSearches::ER_1opt_delivery_delta_cost(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, int & unl_pos) {
    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    int orig_node = solution.routes_delivery_[orig_vhc].visited_nodes_[orig_pos];
    int orig_req = orig_node - num_reqs;
    double orig_vhc_route_cost, dest_vhc_route_cost;
    double orig_vhc_diff_route_cost, dest_vhc_diff_route_cost;
    double orig_vhc_unload_end, dest_vhc_unload_end;
    vector<double> start_reload_avail_gap(num_reqs + 1, -1.0);

    // delta cost on routes
    orig_vhc_route_cost = solution.routes_delivery_[orig_vhc].cost_
                          - Instance::instance()->EdgeCost(solution.routes_delivery_[orig_vhc].visited_nodes_[orig_pos - 1], orig_node)
                          - Instance::instance()->EdgeCost(orig_node, solution.routes_delivery_[orig_vhc].visited_nodes_[orig_pos + 1])
                          + Instance::instance()->EdgeCost(solution.routes_delivery_[orig_vhc].visited_nodes_[orig_pos - 1], solution.routes_delivery_[orig_vhc].visited_nodes_[orig_pos + 1]);

    dest_vhc_route_cost = solution.routes_delivery_[dest_vhc].cost_
                          + Instance::instance()->EdgeCost(solution.routes_delivery_[dest_vhc].visited_nodes_[dest_pos - 1], orig_node)
                          + Instance::instance()->EdgeCost(orig_node, solution.routes_delivery_[dest_vhc].visited_nodes_[dest_pos])
                          - Instance::instance()->EdgeCost(solution.routes_delivery_[dest_vhc].visited_nodes_[dest_pos - 1], solution.routes_delivery_[dest_vhc].visited_nodes_[dest_pos]);

    orig_vhc_diff_route_cost = orig_vhc_route_cost - solution.routes_delivery_[orig_vhc].cost_;
    dest_vhc_diff_route_cost = dest_vhc_route_cost - solution.routes_delivery_[dest_vhc].cost_;

    delta_cost = orig_vhc_diff_route_cost + dest_vhc_diff_route_cost;

    Schedules & schedules = solution.schedule_manager_.schedules_;

    for (int k = 0; k < num_vhcs; ++k) {
        for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
            start_reload_avail_gap[schedules[k].reloaded_reqs_[i].ind_] = schedules[k].reloaded_reqs_[i].start_time_ - schedules[k].reloaded_reqs_[i].availability_time_;
        }
    }

    orig_vhc_unload_end = schedules[orig_vhc].unload_end_;
    dest_vhc_unload_end = schedules[dest_vhc].unload_end_;

    req_start_unload[orig_req] = 0.0;

    for (uint i = 0; i < schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.size(); ++i) {
        if (schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_[i].ind_ == orig_req) {
            req_start_unload[orig_req] = schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_[i].start_time_;
            break;
        }
    }

    // unloading updates
    double aux_start_unload = -1.0;
    double aux_start_unl_orig_node;
    if (solution.schedule_manager_.pic_del_[orig_req].first == dest_vhc) { // doesn't unload anymore
        bool after_orig_node = false;
        for (uint i = 0; i < schedules[dest_vhc].unloaded_reqs_.size(); ++i) {
            if (schedules[dest_vhc].unloaded_reqs_[i].ind_ == orig_req) {
                aux_start_unload = schedules[dest_vhc].unloaded_reqs_[i].start_time_;
                after_orig_node = true;
            } else if (after_orig_node) {
                req_start_unload[schedules[dest_vhc].unloaded_reqs_[i].ind_] = aux_start_unload;
                aux_start_unload += Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_);
            }
        }
        if (schedules[dest_vhc].unloaded_reqs_.size() == 1) {
            dest_vhc_unload_end = schedules[dest_vhc].start_time_;
        } else {
            dest_vhc_unload_end = aux_start_unload;
        }
    } else if (solution.schedule_manager_.pic_del_[orig_req].first == solution.schedule_manager_.pic_del_[orig_req].second) { // starts unloading
        if (schedules[orig_vhc].unloaded_reqs_.empty()) {
            req_start_unload[orig_req] = schedules[orig_vhc].start_time_ + Instance::instance()->unload_preparation_time();
            orig_vhc_unload_end = req_start_unload[orig_req] + Instance::instance()->unloading_time(orig_req);
            unl_pos = 0;
        } else {
            aux_start_unl_orig_node = orig_vhc_unload_end;
            orig_vhc_unload_end += Instance::instance()->unloading_time(orig_req);
            unl_pos = schedules[orig_vhc].unloaded_reqs_.size();
            for (int i = schedules[orig_vhc].unloaded_reqs_.size() - 1; i >= 0; --i) {
                if (cmp(Instance::instance()->unloading_time(orig_req), start_reload_avail_gap[schedules[orig_vhc].unloaded_reqs_[i].ind_]) <= 0) {
                    aux_start_unl_orig_node -= Instance::instance()->unloading_time(schedules[orig_vhc].unloaded_reqs_[i].ind_);
                    req_start_unload[schedules[orig_vhc].unloaded_reqs_[i].ind_] = schedules[orig_vhc].unloaded_reqs_[i].start_time_ + Instance::instance()->unloading_time(orig_req);
                    unl_pos = i;
                } else {
                    break;
                }
            }
            req_start_unload[orig_req] = aux_start_unl_orig_node;
        }
    } else { // changes the reload vehicle
        if (cmp(schedules[orig_vhc].unload_end_, schedules[dest_vhc].unload_end_) < 0) {
            int unload_vhc = solution.schedule_manager_.pic_del_[orig_req].first;
            double req_start_reload = 0.0;
            bool after_orig_req = false;
            double dest_vhc_reload_start = schedules[dest_vhc].unload_end_ + Instance::instance()->reload_preparation_time();
            for (uint i = 0; i < schedules[unload_vhc].unloaded_reqs_.size(); ++i) {
                if (schedules[unload_vhc].unloaded_reqs_[i].ind_ == orig_req) {
                    unl_pos = i;
                    req_start_reload = schedules[unload_vhc].unloaded_reqs_[i].start_time_
                                       + Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_)
                                       + Instance::instance()->between_docks_time(unload_vhc, dest_vhc);
                    after_orig_req = true;
                } else if (after_orig_req) {
                    if (cmp(req_start_reload + Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_), dest_vhc_reload_start) <= 0) {
                        req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_] = schedules[unload_vhc].unloaded_reqs_[i].start_time_ - Instance::instance()->unloading_time(orig_req);
                        req_start_unload[orig_req] = req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_] + Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                        req_start_reload += Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                        unl_pos = i;
                    } else {
                        break;
                    }
                }
            }

        } else {
            int unload_vhc = solution.schedule_manager_.pic_del_[orig_req].first;
            bool before_orig_req = false;
            for (int i = schedules[unload_vhc].unloaded_reqs_.size() - 1; i >= 0; --i) {
                if (schedules[unload_vhc].unloaded_reqs_[i].ind_ == orig_req) {
                    unl_pos = i;
                    before_orig_req = true;
                } else if (before_orig_req) {
                    if (cmp(Instance::instance()->unloading_time(orig_req), start_reload_avail_gap[schedules[unload_vhc].unloaded_reqs_[i].ind_]) <= 0) {
                        req_start_unload[orig_req] = schedules[unload_vhc].unloaded_reqs_[i].start_time_;
                        req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_] = schedules[unload_vhc].unloaded_reqs_[i].start_time_ + Instance::instance()->unloading_time(orig_req);
                        unl_pos = i;
                    } else {
                        break;
                    }
                }
            }
        }
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(orig_vhc);
    impacted_reloads.insert(dest_vhc);

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
                if (rs[i].ind_ != orig_req) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                } else {
                    rs.erase(rs.begin() + i);
                    --i;
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == dest_vhc) {
            curr_unload_end = dest_vhc_unload_end;
            if (solution.schedule_manager_.pic_del_[orig_req].first != dest_vhc) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = orig_req;
                rs[rs.size() - 1].availability_time_ = req_start_unload[orig_req] + Instance::instance()->unloading_time(orig_req) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[orig_req].first, *rld_vhc);
            }
        } else if (*rld_vhc == orig_vhc) {
            curr_unload_end = orig_vhc_unload_end;
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

void LocalSearches::ER_move_1opt_pickup_routes(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, const int unl_pos) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    int orig_node = solution.routes_pickup_[orig_vhc].visited_nodes_[orig_pos];

    // delta cost on routes
    solution.routes_pickup_[orig_vhc].RemoveVisitedNodeFromPos(orig_pos);

    solution.routes_pickup_[dest_vhc].InsertVisitedNode(dest_pos, orig_node);

    Schedules & schedules = solution.schedule_manager_.schedules_;

    // unloading updates
    if (solution.schedule_manager_.pic_del_[orig_node].first != solution.schedule_manager_.pic_del_[orig_node].second) {
        for (uint i = 0; i < schedules[orig_vhc].unloaded_reqs_.size(); ++i) {
            if (schedules[orig_vhc].unloaded_reqs_[i].ind_ == orig_node) {
                schedules[orig_vhc].unloaded_reqs_.erase(schedules[orig_vhc].unloaded_reqs_.begin() + i);
                break;
            }
        }
        solution.schedule_manager_.pic_del_[orig_node].first = dest_vhc;

        if (solution.schedule_manager_.pic_del_[orig_node].first != solution.schedule_manager_.pic_del_[orig_node].second) {
            schedules[dest_vhc].unloaded_reqs_.insert(schedules[dest_vhc].unloaded_reqs_.begin() + unl_pos, UnloadUnit(orig_node));
        } else {
            for (uint i = 0; i < schedules[dest_vhc].reloaded_reqs_.size(); ++i) {
                if (schedules[dest_vhc].reloaded_reqs_[i].ind_ == orig_node) {
                    schedules[dest_vhc].reloaded_reqs_.erase(schedules[dest_vhc].reloaded_reqs_.begin() + i);
                    break;
                }
            }
        }
    } else {
        solution.schedule_manager_.pic_del_[orig_node].first = dest_vhc;
        schedules[dest_vhc].unloaded_reqs_.insert(schedules[dest_vhc].unloaded_reqs_.begin() + unl_pos, UnloadUnit(orig_node));
        schedules[solution.schedule_manager_.pic_del_[orig_node].second].reloaded_reqs_.push_back(ReloadUnit(orig_node));
    }

    schedules[orig_vhc].start_time_ = solution.routes_pickup_[orig_vhc].cost_;
    schedules[dest_vhc].start_time_ = solution.routes_pickup_[dest_vhc].cost_;

    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(orig_vhc);
    impacted_reloads.insert(dest_vhc);

    aux_start_unload = schedules[orig_vhc].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[orig_vhc].unloaded_reqs_.size(); ++i) {
        schedules[orig_vhc].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[orig_vhc].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[orig_vhc].unloaded_reqs_[i].ind_) {
                
                schedules[solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[orig_vhc].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[orig_vhc].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(orig_vhc, solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[orig_vhc].unloaded_reqs_.empty()) {
        schedules[orig_vhc].unload_end_ = schedules[orig_vhc].start_time_;
    } else {
        schedules[orig_vhc].unload_end_ = aux_start_unload;
    }

    aux_start_unload = schedules[dest_vhc].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[dest_vhc].unloaded_reqs_.size(); ++i) {
        schedules[dest_vhc].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[dest_vhc].unloaded_reqs_[i].ind_) {
                
                schedules[solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[dest_vhc].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(dest_vhc, solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[dest_vhc].unloaded_reqs_.empty()) {
        schedules[dest_vhc].unload_end_ = schedules[dest_vhc].start_time_;
    } else {
        schedules[dest_vhc].unload_end_ = aux_start_unload;
    }

    double infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

            schedules[*it].completion_time_ = schedules[*it].reloaded_reqs_.back().start_time_ + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_.back().ind_);
        } else {
            schedules[*it].completion_time_ = schedules[*it].unload_end_;
        }
    }

    solution.cost_ = 0.0;
    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution.cost_ += solution.routes_delivery_[k].cost_ + schedules[k].completion_time_;
    }
}

void LocalSearches::ER_move_1opt_delivery_routes(Solution & solution, const int orig_vhc, const int orig_pos, const int dest_vhc, const int dest_pos, const int unl_pos) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    int orig_node = solution.routes_delivery_[orig_vhc].visited_nodes_[orig_pos];
    int orig_req = orig_node - num_reqs;

    // delta cost on routes
    solution.routes_delivery_[orig_vhc].RemoveVisitedNodeFromPos(orig_pos);

    solution.routes_delivery_[dest_vhc].InsertVisitedNode(dest_pos, orig_node);

    Schedules & schedules = solution.schedule_manager_.schedules_;

    // unloading updates
    if (solution.schedule_manager_.pic_del_[orig_req].first != solution.schedule_manager_.pic_del_[orig_req].second) { // pic != del before change del_vhc
        for (uint i = 0; i < schedules[orig_vhc].reloaded_reqs_.size(); ++i) {
            if (schedules[orig_vhc].reloaded_reqs_[i].ind_ == orig_req) {
                schedules[orig_vhc].reloaded_reqs_.erase(schedules[orig_vhc].reloaded_reqs_.begin() + i);
                break;
            }
        }
        solution.schedule_manager_.pic_del_[orig_req].second = dest_vhc;

        if (solution.schedule_manager_.pic_del_[orig_req].first != solution.schedule_manager_.pic_del_[orig_req].second) { // pic != del after change del_vhc
            schedules[dest_vhc].reloaded_reqs_.push_back(ReloadUnit(orig_req));
            for (int i = 0; i < static_cast<int>(schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.size()); ++i) {
                if (schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_[i].ind_ == orig_req) {
                    if (i != unl_pos) {
                        schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.erase(schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.begin() + i);
                        schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.begin() + unl_pos, UnloadUnit(orig_req));
                    }
                    break;
                }
            }
        } else { // pic == del after change del_vhc
            for (uint i = 0; i < schedules[dest_vhc].unloaded_reqs_.size(); ++i) {
                if (schedules[dest_vhc].unloaded_reqs_[i].ind_ == orig_req) {
                    schedules[dest_vhc].unloaded_reqs_.erase(schedules[dest_vhc].unloaded_reqs_.begin() + i);
                    break;
                }
            }
        }
    } else { // pic == del before change del_vhc
        solution.schedule_manager_.pic_del_[orig_req].second = dest_vhc;
        schedules[dest_vhc].reloaded_reqs_.push_back(ReloadUnit(orig_req));
        schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[orig_req].first].unloaded_reqs_.begin() + unl_pos, UnloadUnit(orig_req));
    }

    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(orig_vhc);
    impacted_reloads.insert(dest_vhc);

    if (solution.schedule_manager_.pic_del_[orig_req].first == orig_vhc) { // starts unloading orig_req in orig_vhc
        aux_start_unload = schedules[orig_vhc].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[orig_vhc].unloaded_reqs_.size(); ++i) {
            schedules[orig_vhc].unloaded_reqs_[i].start_time_ = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[orig_vhc].unloaded_reqs_[i].ind_);
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second);

            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                    == schedules[orig_vhc].unloaded_reqs_[i].ind_) {
                    
                    schedules[solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                            schedules[orig_vhc].unloaded_reqs_[i].start_time_
                            + Instance::instance()->unloading_time(schedules[orig_vhc].unloaded_reqs_[i].ind_)
                            + Instance::instance()->between_docks_time(orig_vhc, solution.schedule_manager_.pic_del_[schedules[orig_vhc].unloaded_reqs_[i].ind_].second);
                }
            }
        }
        if (schedules[orig_vhc].unloaded_reqs_.empty()) {
            schedules[orig_vhc].unload_end_ = schedules[orig_vhc].start_time_;
        } else {
            schedules[orig_vhc].unload_end_ = aux_start_unload;
        }
    } else if (solution.schedule_manager_.pic_del_[orig_req].first == dest_vhc) { // stops unloading orig_req in dest_vhc
        aux_start_unload = schedules[dest_vhc].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[dest_vhc].unloaded_reqs_.size(); ++i) {
            schedules[dest_vhc].unloaded_reqs_[i].start_time_ = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_);
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second);

            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                    == schedules[dest_vhc].unloaded_reqs_[i].ind_) {
                    
                    schedules[solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                            schedules[dest_vhc].unloaded_reqs_[i].start_time_
                            + Instance::instance()->unloading_time(schedules[dest_vhc].unloaded_reqs_[i].ind_)
                            + Instance::instance()->between_docks_time(dest_vhc, solution.schedule_manager_.pic_del_[schedules[dest_vhc].unloaded_reqs_[i].ind_].second);
                }
            }
        }
        if (schedules[dest_vhc].unloaded_reqs_.empty()) {
            schedules[dest_vhc].unload_end_ = schedules[dest_vhc].start_time_;
        } else {
            schedules[dest_vhc].unload_end_ = aux_start_unload;
        }
    } else {
        int pic_vhc = solution.schedule_manager_.pic_del_[orig_req].first;
        impacted_reloads.insert(pic_vhc);
        aux_start_unload = schedules[pic_vhc].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[pic_vhc].unloaded_reqs_.size(); ++i) {
            schedules[pic_vhc].unloaded_reqs_[i].start_time_ = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[pic_vhc].unloaded_reqs_[i].ind_);
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[pic_vhc].unloaded_reqs_[i].ind_].second);

            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[pic_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[schedules[pic_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                    == schedules[pic_vhc].unloaded_reqs_[i].ind_) {
                    
                    schedules[solution.schedule_manager_.pic_del_[schedules[pic_vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                            schedules[pic_vhc].unloaded_reqs_[i].start_time_
                            + Instance::instance()->unloading_time(schedules[pic_vhc].unloaded_reqs_[i].ind_)
                            + Instance::instance()->between_docks_time(pic_vhc, solution.schedule_manager_.pic_del_[schedules[pic_vhc].unloaded_reqs_[i].ind_].second);
                }
            }
        }
        if (schedules[pic_vhc].unloaded_reqs_.empty()) {
            schedules[pic_vhc].unload_end_ = schedules[pic_vhc].start_time_;
        } else {
            schedules[pic_vhc].unload_end_ = aux_start_unload;
        }
    }

    double infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

            schedules[*it].completion_time_ = schedules[*it].reloaded_reqs_.back().start_time_ + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_.back().ind_);
        } else {
            schedules[*it].completion_time_ = schedules[*it].unload_end_;
        }
    }

    solution.cost_ = 0.0;
    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution.cost_ += solution.routes_delivery_[k].cost_ + schedules[k].completion_time_;
    }
}

bool LocalSearches::ER_2opt(Solution & solution, LS_strat search_strat) {

    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    bool improve, general_improvement, is_pickup, best_is_pickup, best_is_b_move, change_vhcs_a_mov, best_change_vhcs_a_mov = false;
    int node_1, pos_1, k1, best_pos_1 = -1, best_pos_2 = -1, best_vhc_1 = -1, best_vhc_2 = -1, req_1;
    int k1_part1_load, k1_part2_load, k2_part1_load, k2_part2_load;
    int for_begin, for_end_offset, for_begin_b, for_end_b_offset;
    double best_cost, temp_cost, current_cost;
    map<int, set<int>> edges_already_used_a;
    int curr_edge, j_edge, num_total_nodes = 2 * num_requests + 1;
    map<int, vector<int>> vhc_unload_order, best_vhc_unload_order;

    vector<int> selectable_nodes(2 * num_requests + 2 * num_vehicles); // cada num alem de num_req representa o CD de um vhc. [num_req + 1 == CD do vhc 0] [num_req + 2 == CD do vhc 1] .. [num_req + num_vhc == CD do vhc num_vhc-1]
    iota(selectable_nodes.begin(), selectable_nodes.end(), 1);
    shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;
    while (improve) {
        improve = false;
        best_cost = solution.cost_;
        edges_already_used_a.clear();
        for (auto selected_node = selectable_nodes.begin(); selected_node != selectable_nodes.end(); ++selected_node) {

            if (*selected_node > 2 * num_requests) {
                node_1 = 0;
                is_pickup = *selected_node <= 2 * num_requests + num_vehicles ? true : false;
                k1 = is_pickup ? *selected_node - 2 * num_requests - 1 : *selected_node - 2 * num_requests - num_vehicles - 1;
            } else {
                node_1 = *selected_node;
                is_pickup = *selected_node <= num_requests ? true : false;
                req_1 = is_pickup ? *selected_node : *selected_node - num_requests;
                k1 = is_pickup ? solution.schedule_manager_.pic_del_[req_1].first : solution.schedule_manager_.pic_del_[req_1].second;
            }
            Routes & routes = is_pickup ? solution.routes_pickup_ : solution.routes_delivery_;

            pos_1 = find(routes[k1].visited_nodes_.begin(), routes[k1].visited_nodes_.end(), node_1) - routes[k1].visited_nodes_.begin();

            if (pos_1 == 0) {
                for_begin = 1; // faz diferenca (ao inves de comecar em 2), pois a troca de rotas entre veiculos pode alterar custos do sequenciamento
                for_end_offset = 0;

                for_begin_b = 1;
                for_end_b_offset = -1;//0;
            } else if (pos_1 == routes[k1].num_nodes_) {
                for_begin = 2;
                for_end_offset = 0;

                for_begin_b = 1;
                for_end_b_offset = -1;
            } else {
                for_begin = 1;
                for_end_offset = 1;

                for_begin_b = 0;
                for_end_b_offset = 0;
            }

            k1_part1_load = 0;
            for (int i = 1; i <= pos_1; ++i) {
                k1_part1_load += Instance::instance()->node(routes[k1].visited_nodes_[i]).demand();
            }
            k1_part2_load = routes[k1].load_ - k1_part1_load;

            temp_cost = solution.cost_;
            curr_edge = routes[k1].visited_nodes_[pos_1] * num_total_nodes + routes[k1].visited_nodes_[pos_1+1];

            for (int k2 = 0; k2 < num_vehicles; ++k2) {
                if (k1 != k2) {
                    k2_part1_load = 0;
                    k2_part2_load = routes[k2].load_;
                    for (int j = for_begin; j <= routes[k2].num_nodes_ + for_end_offset; ++j) {
                        k2_part1_load += Instance::instance()->node(routes[k2].visited_nodes_[j-1]).demand();
                        k2_part2_load -= Instance::instance()->node(routes[k2].visited_nodes_[j-1]).demand();
                        if (k1_part1_load + k2_part2_load <= vhc_capacity && k1_part2_load + k2_part1_load <= vhc_capacity) {

                            j_edge = routes[k2].visited_nodes_[j-1] * num_total_nodes + routes[k2].visited_nodes_[j];
                            vhc_unload_order.clear();
                            current_cost = temp_cost;

                            if (edges_already_used_a[curr_edge].find(j_edge) == edges_already_used_a[curr_edge].end()) {
                                edges_already_used_a[j_edge].insert(curr_edge);
                                current_cost += (is_pickup ? ER_2opt_pickup_delta_cost(solution, k1, pos_1, k2, j, vhc_unload_order, false) : ER_2opt_delivery_delta_cost(solution, k1, pos_1, k2, j, vhc_unload_order, false));
                                change_vhcs_a_mov = false;
                            } else if (node_1 != 0 || j != 1) {
                                change_vhcs_a_mov = true;
                                current_cost += (is_pickup ? ER_2opt_pickup_delta_cost(solution, k1, pos_1, k2, j, vhc_unload_order, false, change_vhcs_a_mov) : ER_2opt_delivery_delta_cost(solution, k1, pos_1, k2, j, vhc_unload_order, false, change_vhcs_a_mov));
                            }

                            if (cmp(current_cost, best_cost) < 0) {
                                best_cost = current_cost;
                                best_vhc_1 = k1;
                                best_vhc_2 = k2;
                                best_pos_1 = pos_1;
                                best_pos_2 = j;
                                best_is_pickup = is_pickup;
                                best_is_b_move = false;
                                best_change_vhcs_a_mov = change_vhcs_a_mov;
                                best_vhc_unload_order = vhc_unload_order;
                                improve = true;
                            }
                        }
                    }

                    k2_part1_load = 0;
                    k2_part2_load = routes[k2].load_;
                    for (int j = for_begin_b; j <= routes[k2].num_nodes_ + for_end_b_offset; ++j) {
                        k2_part1_load += Instance::instance()->node(routes[k2].visited_nodes_[j]).demand();
                        k2_part2_load -= Instance::instance()->node(routes[k2].visited_nodes_[j]).demand();
                        if (k1_part1_load + k2_part1_load <= vhc_capacity && k1_part2_load + k2_part2_load <= vhc_capacity) {
                            vhc_unload_order.clear();
                            current_cost = temp_cost + (is_pickup ? ER_2opt_pickup_delta_cost(solution, k1, pos_1, k2, j,vhc_unload_order, true) : ER_2opt_delivery_delta_cost(solution, k1, pos_1, k2, j, vhc_unload_order, true));

                            if (cmp(current_cost, best_cost) < 0) {
                                best_cost = current_cost;
                                best_vhc_1 = k1;
                                best_vhc_2 = k2;
                                best_pos_1 = pos_1;
                                best_pos_2 = j;
                                best_is_pickup = is_pickup;
                                best_is_b_move = true;
                                best_vhc_unload_order = vhc_unload_order;
                                improve = true;
                            }
                        }
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;
                if (best_is_pickup) {
                    ER_move_2opt_pickup_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_vhc_unload_order, best_is_b_move, best_change_vhcs_a_mov);
                } else {
                    ER_move_2opt_delivery_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_vhc_unload_order, best_is_b_move, best_change_vhcs_a_mov);
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
                ER_move_2opt_pickup_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_vhc_unload_order, best_is_b_move, best_change_vhcs_a_mov);
            } else {
                ER_move_2opt_delivery_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_vhc_unload_order, best_is_b_move, best_change_vhcs_a_mov);
            }

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }
    }

    return general_improvement;
}

double LocalSearches::ER_2opt_pickup_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov) {
    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    double vhc_1_route_cost, vhc_2_route_cost;
    double vhc_1_unload_end, vhc_2_unload_end;
    set<int> reqs_in_vhc_1, reqs_in_vhc_2;
    vector<int> & unload_vhc_1 = vhc_unload_order[vhc_1];
    vector<int> & unload_vhc_2 = vhc_unload_order[vhc_2];
    int old_unl_size_vhc_1 = 0, old_unl_size_vhc_2 = 0, first_new_unl_in_vhc_1 = -1, first_new_unl_in_vhc_2 = -1;

    // delta cost on routes
    vhc_1_route_cost = 0.0;
    vhc_2_route_cost = 0.0;

    if (is_b_mov) {
        for (int i = 0; i < pos_1; ++i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i], solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
            reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
        }
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_2]);
        for (int i = pos_2; i > 0; --i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i], solution.routes_pickup_[vhc_2].visited_nodes_[i-1]);
            reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[i]);
        }
        for (int i = solution.routes_pickup_[vhc_1].num_nodes_; i >= pos_1+1; --i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i+1], solution.routes_pickup_[vhc_1].visited_nodes_[i]);
            reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_1].visited_nodes_[i]);
        }
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_1+1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_2+1]);
        for (int i = pos_2+1; i <= solution.routes_pickup_[vhc_2].num_nodes_; ++i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i], solution.routes_pickup_[vhc_2].visited_nodes_[i+1]);
            reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_2].visited_nodes_[i]);
        }
    } else {
        for (int i = 0; i < pos_1; ++i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i], solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
            reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
        }
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_2]);
        for (int i = pos_2; i <= solution.routes_pickup_[vhc_2].num_nodes_; ++i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i], solution.routes_pickup_[vhc_2].visited_nodes_[i+1]);
            reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[i]);
        }
        for (int i = 0; i < pos_2 - 1; ++i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i], solution.routes_pickup_[vhc_2].visited_nodes_[i+1]);
            reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_2].visited_nodes_[i+1]);
        }
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_2-1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_1+1]);
        for (int i = pos_1+1; i <= solution.routes_pickup_[vhc_1].num_nodes_; ++i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i], solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
            reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_1].visited_nodes_[i]);
        }

        if (change_vhcs_a_mov) {
            swap(vhc_1_route_cost, vhc_2_route_cost);
            swap(reqs_in_vhc_1, reqs_in_vhc_2);
        }
    }

    // unloading updates
    double aux_start_unload;
    multimap<int, int> new_reqs_in_vhc_1;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    // on vhc_1
    aux_start_unload = vhc_1_route_cost + Instance::instance()->unload_preparation_time();
    // 1st: schedule the requests that were already unloading in vhc_1
    for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
        if (reqs_in_vhc_1.find(schedules[vhc_1].unloaded_reqs_[i].ind_) != reqs_in_vhc_1.end()) {
            req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
            unload_vhc_1.push_back(schedules[vhc_1].unloaded_reqs_[i].ind_);
        }
    }
    old_unl_size_vhc_1 = static_cast<int>(unload_vhc_1.size());
    // 2nd: schedule the requests that were unloaded in vhc_2
    // (besides that, set with 0.0 the requests that are not unloaded anymore)
    for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
        if (reqs_in_vhc_1.find(schedules[vhc_2].unloaded_reqs_[i].ind_) != reqs_in_vhc_1.end()) {
            if (solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second != vhc_1) {
                req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] = aux_start_unload;
                aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
                unload_vhc_1.push_back(schedules[vhc_2].unloaded_reqs_[i].ind_);
            } else {
                req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] = 0.0;
            }
        }
    }
    // 3rd: schedule (in increasing order) the requests that were not unloaded in vhc_2 but need to be unloaded in vhc_1
    for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
        if (req_start_unload[*it] < 0.0) {
            if (solution.schedule_manager_.pic_del_[*it].second != vhc_1) {
                new_reqs_in_vhc_1.insert(make_pair(Instance::instance()->unloading_time(*it), *it));
            }
        }
    }
    for (auto it = new_reqs_in_vhc_1.begin(); it != new_reqs_in_vhc_1.end(); ++it) {
        req_start_unload[it->second] = aux_start_unload;
        aux_start_unload += it->first;
        unload_vhc_1.push_back(it->second);
    }

    if (!unload_vhc_1.empty()) {
        vhc_1_unload_end = aux_start_unload;
        if (static_cast<int>(unload_vhc_1.size()) > old_unl_size_vhc_1) {
            first_new_unl_in_vhc_1 = old_unl_size_vhc_1;
        }
    } else {
        vhc_1_unload_end = vhc_1_route_cost;
    }

    // on vhc_2
    aux_start_unload = vhc_2_route_cost + Instance::instance()->unload_preparation_time();
    multimap<int, int> new_reqs_in_vhc_2;
    // 1st: schedule the requests that were already unloading in vhc_2
    for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
        if (reqs_in_vhc_2.find(schedules[vhc_2].unloaded_reqs_[i].ind_) != reqs_in_vhc_2.end()) {
            req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
            unload_vhc_2.push_back(schedules[vhc_2].unloaded_reqs_[i].ind_);
        }
    }
    old_unl_size_vhc_2 = static_cast<int>(unload_vhc_2.size());
    // 2nd: schedule the requests that were unloaded in vhc_1
    // (besides that, set with 0.0 the requests that are not unloaded anymore)
    for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
        if (reqs_in_vhc_2.find(schedules[vhc_1].unloaded_reqs_[i].ind_) != reqs_in_vhc_2.end()) {
            if (solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second != vhc_2) {
                req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] = aux_start_unload;
                aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
                unload_vhc_2.push_back(schedules[vhc_1].unloaded_reqs_[i].ind_);
            } else {
                req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] = 0.0;
            }
        }
    }
    // 3rd: schedule (in increasing order) the requests that were not unloaded in vhc_1 but need to be unloaded in vhc_2
    for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
        if (req_start_unload[*it] < 0.0) {
            if (solution.schedule_manager_.pic_del_[*it].second != vhc_2) {
                new_reqs_in_vhc_2.insert(make_pair(Instance::instance()->unloading_time(*it), *it));
            }
        }
    }
    for (auto it = new_reqs_in_vhc_2.begin(); it != new_reqs_in_vhc_2.end(); ++it) {
        req_start_unload[it->second] = aux_start_unload;
        aux_start_unload += it->first;
        unload_vhc_2.push_back(it->second);
    }

    if (!unload_vhc_2.empty()) {
        vhc_2_unload_end = aux_start_unload;
        if (static_cast<int>(unload_vhc_2.size()) > old_unl_size_vhc_2) {
            first_new_unl_in_vhc_2 = old_unl_size_vhc_2;
        }
    } else {
        vhc_2_unload_end = vhc_2_route_cost;
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_unload_end, infeas_st_av_size;

    vector<double> start_reload(num_reqs + 1, -1.0), ct_delta(num_vhcs, 0.0);

    for (int k = 0; k < num_vhcs; ++k) {
        for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
            start_reload[schedules[k].reloaded_reqs_[i].ind_] = schedules[k].reloaded_reqs_[i].start_time_;
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (reqs_in_vhc_1.find(rs[i].ind_) != reqs_in_vhc_1.end()) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_1) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(vhc_1, *rld_vhc);
                    }
                } else if (reqs_in_vhc_2.find(rs[i].ind_) != reqs_in_vhc_2.end()) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(vhc_2, *rld_vhc);
                    }
                } else {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_2 && solution.schedule_manager_.pic_del_[*it].second == vhc_1) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(vhc_2, *rld_vhc);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_1 && solution.schedule_manager_.pic_del_[*it].second == vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(vhc_1, *rld_vhc);
                }
            }
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
                    start_reload[rs[i].ind_] = rs[i].start_time_;
                }
            }

            ct_delta[*rld_vhc] = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
        } else {
            ct_delta[*rld_vhc] = curr_unload_end - schedules[*rld_vhc].completion_time_;
        }
    }

    impacted_reloads.clear();

    int current_req;
    for (uint i = first_new_unl_in_vhc_1; i < unload_vhc_1.size(); ++i) {
        current_req = unload_vhc_1[i];
        for (int j = i; j > 0; --j) {
            if (cmp(Instance::instance()->unloading_time(current_req),
                    start_reload[unload_vhc_1[j-1]]
                    - (req_start_unload[unload_vhc_1[j-1]]
                       + Instance::instance()->unloading_time(unload_vhc_1[j-1])
                       + Instance::instance()->between_docks_time(vhc_1, solution.schedule_manager_.pic_del_[unload_vhc_1[j-1]].second))) <= 0) {
                req_start_unload[current_req] -= Instance::instance()->unloading_time(unload_vhc_1[j-1]);
                req_start_unload[unload_vhc_1[j-1]] += Instance::instance()->unloading_time(current_req);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[unload_vhc_1[j-1]].second);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[current_req].second);
                swap(unload_vhc_1[j-1], unload_vhc_1[j]);
            } else {
                break;
            }
        }
    }

    for (uint i = first_new_unl_in_vhc_2; i < unload_vhc_2.size(); ++i) {
        current_req = unload_vhc_2[i];
        for (int j = i; j > 0; --j) {
            if (cmp(Instance::instance()->unloading_time(current_req),
                    start_reload[unload_vhc_2[j-1]]
                    - (req_start_unload[unload_vhc_2[j-1]]
                       + Instance::instance()->unloading_time(unload_vhc_2[j-1])
                       + Instance::instance()->between_docks_time(vhc_2, solution.schedule_manager_.pic_del_[unload_vhc_2[j-1]].second))) <= 0) {
                req_start_unload[current_req] -= Instance::instance()->unloading_time(unload_vhc_2[j-1]);
                req_start_unload[unload_vhc_2[j-1]] += Instance::instance()->unloading_time(current_req);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[unload_vhc_2[j-1]].second);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[current_req].second);
                swap(unload_vhc_2[j-1], unload_vhc_2[j]);
            } else {
                break;
            }
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (reqs_in_vhc_1.find(rs[i].ind_) != reqs_in_vhc_1.end()) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_1) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(vhc_1, *rld_vhc);
                    }
                } else if (reqs_in_vhc_2.find(rs[i].ind_) != reqs_in_vhc_2.end()) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(vhc_2, *rld_vhc);
                    }
                } else {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_2 && solution.schedule_manager_.pic_del_[*it].second == vhc_1) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(vhc_2, *rld_vhc);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_1 && solution.schedule_manager_.pic_del_[*it].second == vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(vhc_1, *rld_vhc);
                }
            }
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

            ct_delta[*rld_vhc] = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
        } else {
            ct_delta[*rld_vhc] = curr_unload_end - schedules[*rld_vhc].completion_time_;
        }
    }

    for (int k = 0; k < num_vhcs; ++k) {
        delta_cost += ct_delta[k];
    }

    return delta_cost;
}

double LocalSearches::ER_2opt_delivery_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov) {
    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    double vhc_1_route_cost, vhc_2_route_cost;
    double vhc_1_diff_route_cost, vhc_2_diff_route_cost;
    double vhc_1_unload_end, vhc_2_unload_end;
    set<int> reqs_in_vhc_1, reqs_in_vhc_2;
    int old_unl_size_vhc_1 = 0, old_unl_size_vhc_2 = 0, first_new_unl_in_vhc_1 = -1, first_new_unl_in_vhc_2 = -1;
    set<int> others_reqs_affecteds, others_vhcs_affecteds;
    vector<pair<int, int>> aux_pic_del = solution.schedule_manager_.pic_del_;

    // delta cost on routes
    vhc_1_route_cost = 0.0;
    vhc_2_route_cost = 0.0;

    if (is_b_mov) {
        for (int i = 0; i < pos_1; ++i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i], solution.routes_delivery_[vhc_1].visited_nodes_[i+1]);
            reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_1].visited_nodes_[i+1] - num_reqs);
        }
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_2]);
        for (int i = pos_2; i > 0; --i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i], solution.routes_delivery_[vhc_2].visited_nodes_[i-1]);
            reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs);
            aux_pic_del[solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs].second = vhc_1;
        }
        for (int i = solution.routes_delivery_[vhc_1].num_nodes_; i >= pos_1+1; --i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i+1], solution.routes_delivery_[vhc_1].visited_nodes_[i]);
            reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_1].visited_nodes_[i] - num_reqs);
            aux_pic_del[solution.routes_delivery_[vhc_1].visited_nodes_[i] - num_reqs].second = vhc_2;
        }
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_1+1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_2+1]);
        for (int i = pos_2+1; i <= solution.routes_delivery_[vhc_2].num_nodes_; ++i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i], solution.routes_delivery_[vhc_2].visited_nodes_[i+1]);
            reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs);
        }
    } else {
        for (int i = 0; i < pos_1; ++i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i], solution.routes_delivery_[vhc_1].visited_nodes_[i+1]);
            reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_1].visited_nodes_[i+1] - num_reqs);
            aux_pic_del[solution.routes_delivery_[vhc_1].visited_nodes_[i+1] - num_reqs].second = change_vhcs_a_mov ? vhc_2 : vhc_1;
        }
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_2]);
        for (int i = pos_2; i <= solution.routes_delivery_[vhc_2].num_nodes_; ++i) {
            vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i], solution.routes_delivery_[vhc_2].visited_nodes_[i+1]);
            reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs);
            aux_pic_del[solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs].second = change_vhcs_a_mov ? vhc_2 : vhc_1;
        }

        for (int i = 0; i < pos_2 - 1; ++i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i], solution.routes_delivery_[vhc_2].visited_nodes_[i+1]);
            reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_2].visited_nodes_[i+1] - num_reqs);
            aux_pic_del[solution.routes_delivery_[vhc_2].visited_nodes_[i+1] - num_reqs].second = change_vhcs_a_mov ? vhc_1 : vhc_2;
        }
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_2-1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_1+1]);
        for (int i = pos_1+1; i <= solution.routes_delivery_[vhc_1].num_nodes_; ++i) {
            vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i], solution.routes_delivery_[vhc_1].visited_nodes_[i+1]);
            reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_1].visited_nodes_[i] - num_reqs);
            aux_pic_del[solution.routes_delivery_[vhc_1].visited_nodes_[i] - num_reqs].second = change_vhcs_a_mov ? vhc_1 : vhc_2;
        }

        if (change_vhcs_a_mov) {
            swap(vhc_1_route_cost, vhc_2_route_cost);
            swap(reqs_in_vhc_1, reqs_in_vhc_2);
        }
    }

    vhc_1_diff_route_cost = vhc_1_route_cost - solution.routes_delivery_[vhc_1].cost_;
    vhc_2_diff_route_cost = vhc_2_route_cost - solution.routes_delivery_[vhc_2].cost_;

    delta_cost = vhc_1_diff_route_cost + vhc_2_diff_route_cost;

    // unloading updates
    double aux_start_unload;
    multimap<int, int> new_reqs_in_vhc_1;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    // on vhc_1
    aux_start_unload = schedules[vhc_1].start_time_ + Instance::instance()->unload_preparation_time();
    // 1st: schedule the requests that still unload in vhc_1
    // (besides that, set with 0.0 the requests that are not unloaded anymore)
    for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
        if (reqs_in_vhc_1.find(schedules[vhc_1].unloaded_reqs_[i].ind_) == reqs_in_vhc_1.end()) {
            req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
            vhc_unload_order[vhc_1].push_back(schedules[vhc_1].unloaded_reqs_[i].ind_);
        } else {
            req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] = 0.0;
        }
    }
    old_unl_size_vhc_1 = static_cast<int>(vhc_unload_order[vhc_1].size());
    // 2nd: schedule the requests that were unloaded in vhc_2
    for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
        if (solution.schedule_manager_.pic_del_[*it].first == vhc_1 && solution.schedule_manager_.pic_del_[*it].second == vhc_1) {
            new_reqs_in_vhc_1.insert(make_pair(Instance::instance()->unloading_time(*it), *it));
        }
    }
    for (auto it = new_reqs_in_vhc_1.begin(); it != new_reqs_in_vhc_1.end(); ++it) {
        req_start_unload[it->second] = aux_start_unload;
        aux_start_unload += it->first;
        vhc_unload_order[vhc_1].push_back(it->second);
    }

    if (!vhc_unload_order[vhc_1].empty()) {
        vhc_1_unload_end = aux_start_unload;
        if (static_cast<int>(vhc_unload_order[vhc_1].size()) > old_unl_size_vhc_1) {
            first_new_unl_in_vhc_1 = old_unl_size_vhc_1;
        }
    } else {
        vhc_1_unload_end = schedules[vhc_1].start_time_;
    }

    // on vhc_2
    multimap<int, int> new_reqs_in_vhc_2;
    aux_start_unload = schedules[vhc_2].start_time_ + Instance::instance()->unload_preparation_time();
    // 1st: schedule the requests that still unload in vhc_2
    // (besides that, set with 0.0 the requests that are not unloaded anymore)
    for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
        if (reqs_in_vhc_2.find(schedules[vhc_2].unloaded_reqs_[i].ind_) == reqs_in_vhc_2.end()) {
            req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
            vhc_unload_order[vhc_2].push_back(schedules[vhc_2].unloaded_reqs_[i].ind_);
        } else {
            req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] = 0.0;
        }
    }
    old_unl_size_vhc_2 = static_cast<int>(vhc_unload_order[vhc_2].size());
    // 2nd: schedule the requests that were unloaded in vhc_1
    for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
        if (solution.schedule_manager_.pic_del_[*it].first == vhc_2 && solution.schedule_manager_.pic_del_[*it].second == vhc_2) {
            new_reqs_in_vhc_2.insert(make_pair(Instance::instance()->unloading_time(*it), *it));
        }
    }
    for (auto it = new_reqs_in_vhc_2.begin(); it != new_reqs_in_vhc_2.end(); ++it) {
        req_start_unload[it->second] = aux_start_unload;
        aux_start_unload += it->first;
        vhc_unload_order[vhc_2].push_back(it->second);
    }

    if (!vhc_unload_order[vhc_2].empty()) {
        vhc_2_unload_end = aux_start_unload;
        if (static_cast<int>(vhc_unload_order[vhc_2].size()) > old_unl_size_vhc_2) {
            first_new_unl_in_vhc_2 = old_unl_size_vhc_2;
        }
    } else {
        vhc_2_unload_end = schedules[vhc_2].start_time_;
    }

    // last req_start_unloads
    for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
        if (req_start_unload[*it] < 0.0) {
            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[*it].first].unloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[*it].first].unloaded_reqs_[j].ind_ == *it) {
                    req_start_unload[*it] = schedules[solution.schedule_manager_.pic_del_[*it].first].unloaded_reqs_[j].start_time_;
                    if (solution.schedule_manager_.pic_del_[*it].second == vhc_2) {
                        others_reqs_affecteds.insert(*it);
                        others_vhcs_affecteds.insert(solution.schedule_manager_.pic_del_[*it].first);
                    }
                    break;
                }
            }
        }
    }

    for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
        if (req_start_unload[*it] < 0.0) {
            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[*it].first].unloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[*it].first].unloaded_reqs_[j].ind_ == *it) {
                    req_start_unload[*it] = schedules[solution.schedule_manager_.pic_del_[*it].first].unloaded_reqs_[j].start_time_;
                    if (solution.schedule_manager_.pic_del_[*it].second == vhc_1) {
                        others_reqs_affecteds.insert(*it);
                        others_vhcs_affecteds.insert(solution.schedule_manager_.pic_del_[*it].first);
                    }
                    break;
                }
            }
        }
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_unload_end, infeas_st_av_size;

    vector<double> start_reload(num_reqs + 1, -1.0), ct_delta(num_vhcs, 0.0);

    for (int k = 0; k < num_vhcs; ++k) {
        for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
            start_reload[schedules[k].reloaded_reqs_[i].ind_] = schedules[k].reloaded_reqs_[i].start_time_;
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (reqs_in_vhc_1.find(rs[i].ind_) != reqs_in_vhc_1.end() && solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                } else if (reqs_in_vhc_2.find(rs[i].ind_) != reqs_in_vhc_2.end() && solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_1) { // don't reload anymore
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                } else {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_1 && solution.schedule_manager_.pic_del_[*it].second != vhc_1) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[*it].first, *rld_vhc);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_2 && solution.schedule_manager_.pic_del_[*it].second != vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[*it].first, *rld_vhc);
                }
            }
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
                    start_reload[rs[i].ind_] = rs[i].start_time_;
                }
            }

            ct_delta[*rld_vhc] = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
        } else {
            ct_delta[*rld_vhc] = curr_unload_end - schedules[*rld_vhc].completion_time_;
        }
    }

    impacted_reloads.clear();

    int current_req;
    for (uint i = first_new_unl_in_vhc_1; i < vhc_unload_order[vhc_1].size(); ++i) {
        current_req = vhc_unload_order[vhc_1][i];
        for (int j = i; j > 0; --j) {
            if (cmp(Instance::instance()->unloading_time(current_req),
                    start_reload[vhc_unload_order[vhc_1][j-1]]
                    - (req_start_unload[vhc_unload_order[vhc_1][j-1]]
                       + Instance::instance()->unloading_time(vhc_unload_order[vhc_1][j-1])
                       + Instance::instance()->between_docks_time(vhc_1, aux_pic_del[vhc_unload_order[vhc_1][j-1]].second))) <= 0) {
                req_start_unload[current_req] -= Instance::instance()->unloading_time(vhc_unload_order[vhc_1][j-1]);
                req_start_unload[vhc_unload_order[vhc_1][j-1]] += Instance::instance()->unloading_time(current_req);
                impacted_reloads.insert(aux_pic_del[vhc_unload_order[vhc_1][j-1]].second);
                impacted_reloads.insert(aux_pic_del[current_req].second);
                swap(vhc_unload_order[vhc_1][j-1], vhc_unload_order[vhc_1][j]);
            } else {
                break;
            }
        }
    }

    for (uint i = first_new_unl_in_vhc_2; i < vhc_unload_order[vhc_2].size(); ++i) {
        current_req = vhc_unload_order[vhc_2][i];
        for (int j = i; j > 0; --j) {
            if (cmp(Instance::instance()->unloading_time(current_req),
                    start_reload[vhc_unload_order[vhc_2][j-1]]
                    - (req_start_unload[vhc_unload_order[vhc_2][j-1]]
                       + Instance::instance()->unloading_time(vhc_unload_order[vhc_2][j-1])
                       + Instance::instance()->between_docks_time(vhc_2, aux_pic_del[vhc_unload_order[vhc_2][j-1]].second))) <= 0) {
                req_start_unload[current_req] -= Instance::instance()->unloading_time(vhc_unload_order[vhc_2][j-1]);
                req_start_unload[vhc_unload_order[vhc_2][j-1]] += Instance::instance()->unloading_time(current_req);
                impacted_reloads.insert(aux_pic_del[vhc_unload_order[vhc_2][j-1]].second);
                impacted_reloads.insert(aux_pic_del[current_req].second);
                swap(vhc_unload_order[vhc_2][j-1], vhc_unload_order[vhc_2][j]);
            } else {
                break;
            }
        }
    }

    vector<double> vhc_unload_ends(num_vhcs);
    for (int k = 0; k < num_vhcs; ++k) {
        vhc_unload_ends[k] = schedules[k].unload_end_;
    }
    vhc_unload_ends[vhc_1] = vhc_1_unload_end;
    vhc_unload_ends[vhc_2] = vhc_2_unload_end;

    for (int vhc : others_vhcs_affecteds) {
        for (auto & req : schedules[vhc].unloaded_reqs_) {
            vhc_unload_order[vhc].push_back(req.ind_);
            if (cmp(req_start_unload[req.ind_], 0.0) < 0) {
                req_start_unload[req.ind_] = req.start_time_;
            }
        }
    }

    int old_reload_vhc, new_reload_vhc;
    double req_avail;
    bool was_modified;
    for (int vhc : others_vhcs_affecteds) {
        was_modified = false;
        for (uint i = 0; i < vhc_unload_order[vhc].size(); ++i) {
            if (others_reqs_affecteds.find(vhc_unload_order[vhc][i]) != others_reqs_affecteds.end()) {
                current_req = vhc_unload_order[vhc][i];
                old_reload_vhc = solution.schedule_manager_.pic_del_[current_req].second;
                new_reload_vhc = aux_pic_del[current_req].second;

                if (cmp(vhc_unload_ends[old_reload_vhc], vhc_unload_ends[new_reload_vhc]) < 0) {
                    req_avail = req_start_unload[current_req]
                                + Instance::instance()->unloading_time(current_req)
                                + Instance::instance()->between_docks_time(vhc, new_reload_vhc);
                    for (uint j = i + 1; j < schedules[vhc].unloaded_reqs_.size(); ++j) {
                        if (cmp(req_avail + Instance::instance()->unloading_time(vhc_unload_order[vhc][j]), vhc_unload_ends[new_reload_vhc]) <= 0) {
                            req_start_unload[vhc_unload_order[vhc][j]] -= Instance::instance()->unloading_time(current_req);
                            req_start_unload[current_req] += Instance::instance()->unloading_time(vhc_unload_order[vhc][j]);
                            impacted_reloads.insert(aux_pic_del[vhc_unload_order[vhc][j]].second);
                            impacted_reloads.insert(aux_pic_del[current_req].second);
                            req_avail += Instance::instance()->unloading_time(vhc_unload_order[vhc][j]);
                            swap(vhc_unload_order[vhc][j-1], vhc_unload_order[vhc][j]);
                            was_modified = true;
                        } else {
                            break;
                        }
                    }
                } else {
                    for (int j = i - 1; j >= 0; --j) {
                        if (cmp(Instance::instance()->unloading_time(current_req),
                                start_reload[vhc_unload_order[vhc][j]]
                                - (req_start_unload[vhc_unload_order[vhc][j]]
                                   + Instance::instance()->unloading_time(vhc_unload_order[vhc][j])
                                   + Instance::instance()->between_docks_time(vhc, aux_pic_del[vhc_unload_order[vhc][j]].second))) <= 0) {
                            req_start_unload[current_req] = req_start_unload[vhc_unload_order[vhc][j]];
                            req_start_unload[vhc_unload_order[vhc][j]] += Instance::instance()->unloading_time(current_req);
                            impacted_reloads.insert(aux_pic_del[vhc_unload_order[vhc][j]].second);
                            impacted_reloads.insert(aux_pic_del[current_req].second);
                            swap(vhc_unload_order[vhc][j], vhc_unload_order[vhc][j+1]);
                            was_modified = true;
                        } else {
                            break;
                        }
                    }
                }
            }
        }
        if (!was_modified) {
            vhc_unload_order.erase(vhc);
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (reqs_in_vhc_1.find(rs[i].ind_) != reqs_in_vhc_1.end() && solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                } else if (reqs_in_vhc_2.find(rs[i].ind_) != reqs_in_vhc_2.end() && solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_1) { // don't reload anymore
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                } else {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_1 && solution.schedule_manager_.pic_del_[*it].second != vhc_1) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[*it].first, *rld_vhc);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_2 && solution.schedule_manager_.pic_del_[*it].second != vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[*it].first, *rld_vhc);
                }
            }
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

            ct_delta[*rld_vhc] = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
        } else {
            ct_delta[*rld_vhc] = curr_unload_end - schedules[*rld_vhc].completion_time_;
        }
    }

    for (int k = 0; k < num_vhcs; ++k) {
        delta_cost += ct_delta[k];
    }

    return delta_cost;
}

void LocalSearches::ER_move_2opt_pickup_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    set<int> new_reqs_in_vhc_1, new_reqs_in_vhc_2;

    Route route_vhc_1_bkp = solution.routes_pickup_[vhc_1];

    if (is_b_mov) {
        while (solution.routes_pickup_[vhc_1].visited_nodes_[pos_1 + 1] != 0) {
            solution.routes_pickup_[vhc_1].RemoveVisitedNodeFromPos(pos_1 + 1);
        }
        for (int i = pos_2; i > 0; --i) {
            new_reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[i]);
            solution.schedule_manager_.pic_del_[solution.routes_pickup_[vhc_2].visited_nodes_[i]].first = vhc_1;
            solution.routes_pickup_[vhc_1].InsertVisitedNode(solution.routes_pickup_[vhc_1].last_insertable_pos_, solution.routes_pickup_[vhc_2].visited_nodes_[i]);
        }

        Route route_vhc_2_bkp = solution.routes_pickup_[vhc_2];
        solution.routes_pickup_[vhc_2].Clear();

        for (int i = route_vhc_1_bkp.num_nodes_; i >= pos_1 + 1; --i) {
            new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i]);
            solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i]].first = vhc_2;
            solution.routes_pickup_[vhc_2].InsertVisitedNode(solution.routes_pickup_[vhc_2].last_insertable_pos_, route_vhc_1_bkp.visited_nodes_[i]);
        }
        for (int i = pos_2 + 1; i <= route_vhc_2_bkp.num_nodes_; ++i) {
            solution.routes_pickup_[vhc_2].InsertVisitedNode(solution.routes_pickup_[vhc_2].last_insertable_pos_, route_vhc_2_bkp.visited_nodes_[i]);
        }

    } else if (!change_vhcs_a_mov) {
        while (solution.routes_pickup_[vhc_1].visited_nodes_[pos_1 + 1] != 0) {
            solution.routes_pickup_[vhc_1].RemoveVisitedNodeFromPos(pos_1 + 1);
        }
        for (int i = pos_2; i <= solution.routes_pickup_[vhc_2].num_nodes_; ++i) {
            new_reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[i]);
            solution.schedule_manager_.pic_del_[solution.routes_pickup_[vhc_2].visited_nodes_[i]].first = vhc_1;
            solution.routes_pickup_[vhc_1].InsertVisitedNode(solution.routes_pickup_[vhc_1].last_insertable_pos_, solution.routes_pickup_[vhc_2].visited_nodes_[i]);
        }

        while (solution.routes_pickup_[vhc_2].visited_nodes_[pos_2] != 0) {
            solution.routes_pickup_[vhc_2].RemoveVisitedNodeFromPos(pos_2);
        }
        for (int i = pos_1 + 1; i <= route_vhc_1_bkp.num_nodes_; ++i) {
            new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i]);
            solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i]].first = vhc_2;
            solution.routes_pickup_[vhc_2].InsertVisitedNode(solution.routes_pickup_[vhc_2].last_insertable_pos_, route_vhc_1_bkp.visited_nodes_[i]);
        }
    } else {
        while (solution.routes_pickup_[vhc_1].visited_nodes_[1] != route_vhc_1_bkp.visited_nodes_[pos_1 + 1]) {
            solution.routes_pickup_[vhc_1].RemoveVisitedNodeFromPos(1);
        }
        for (int i = 1; i < pos_2; ++i) {
            new_reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[i]);
            solution.schedule_manager_.pic_del_[solution.routes_pickup_[vhc_2].visited_nodes_[i]].first = vhc_1;
            solution.routes_pickup_[vhc_1].InsertVisitedNode(i, solution.routes_pickup_[vhc_2].visited_nodes_[i]);
        }

        int stop_node = solution.routes_pickup_[vhc_2].visited_nodes_[pos_2];
        while (solution.routes_pickup_[vhc_2].visited_nodes_[1] != stop_node) {
            solution.routes_pickup_[vhc_2].RemoveVisitedNodeFromPos(1);
        }
        for (int i = 1; i <= pos_1; ++i) {
            new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i]);
            solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i]].first = vhc_2;
            solution.routes_pickup_[vhc_2].InsertVisitedNode(i, route_vhc_1_bkp.visited_nodes_[i]);
        }
    }

    Schedules & schedules = solution.schedule_manager_.schedules_;
    schedules[vhc_1].start_time_ = solution.routes_pickup_[vhc_1].cost_;
    schedules[vhc_2].start_time_ = solution.routes_pickup_[vhc_2].cost_;

    // unloading updates
    for (auto & vhc_unl : vhc_unload_order) {
        schedules[vhc_unl.first].unloaded_reqs_.clear();
        for (int r : vhc_unl.second) {
            schedules[vhc_unl.first].unloaded_reqs_.push_back(UnloadUnit(r));
        }
    }

    // reloading updates
    for (uint i = 0; i < schedules[vhc_1].reloaded_reqs_.size(); ++i) {
        if (new_reqs_in_vhc_1.find(schedules[vhc_1].reloaded_reqs_[i].ind_) != new_reqs_in_vhc_1.end()) {
            schedules[vhc_1].reloaded_reqs_.erase(schedules[vhc_1].reloaded_reqs_.begin() + i);
            --i;
        }
    }
    for (auto it = new_reqs_in_vhc_2.begin(); it != new_reqs_in_vhc_2.end(); ++it) {
        if (solution.schedule_manager_.pic_del_[*it].second == vhc_1) {
            schedules[vhc_1].reloaded_reqs_.push_back(ReloadUnit(*it));
        }
    }

    for (uint i = 0; i < schedules[vhc_2].reloaded_reqs_.size(); ++i) {
        if (new_reqs_in_vhc_2.find(schedules[vhc_2].reloaded_reqs_[i].ind_) != new_reqs_in_vhc_2.end()) {
            schedules[vhc_2].reloaded_reqs_.erase(schedules[vhc_2].reloaded_reqs_.begin() + i);
            --i;
        }
    }
    for (auto it = new_reqs_in_vhc_1.begin(); it != new_reqs_in_vhc_1.end(); ++it) {
        if (solution.schedule_manager_.pic_del_[*it].second == vhc_2) {
            schedules[vhc_2].reloaded_reqs_.push_back(ReloadUnit(*it));
        }
    }

    // update times
    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);

    for (int vhc : {vhc_1, vhc_2}) {
        aux_start_unload = schedules[vhc].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[vhc].unloaded_reqs_.size(); ++i) {
            schedules[vhc].unloaded_reqs_[i].start_time_ = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[i].ind_);
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second);

            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                    == schedules[vhc].unloaded_reqs_[i].ind_) {
                    
                    schedules[solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                            schedules[vhc].unloaded_reqs_[i].start_time_
                            + Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[i].ind_)
                            + Instance::instance()->between_docks_time(vhc, solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second);
                }
            }
        }
        if (schedules[vhc].unloaded_reqs_.empty()) {
            schedules[vhc].unload_end_ = schedules[vhc].start_time_;
        } else {
            schedules[vhc].unload_end_ = aux_start_unload;
        }
    }

    double infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

            schedules[*it].completion_time_ = schedules[*it].reloaded_reqs_.back().start_time_ + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_.back().ind_);
        } else {
            schedules[*it].completion_time_ = schedules[*it].unload_end_;
        }
    }

    solution.cost_ = 0.0;
    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution.cost_ += solution.routes_delivery_[k].cost_ + schedules[k].completion_time_;
    }
}

void LocalSearches::ER_move_2opt_delivery_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, map<int, vector<int>> & vhc_unload_order, const bool is_b_mov, const bool change_vhcs_a_mov) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    set<int> new_reqs_in_vhc_1, new_reqs_in_vhc_2;

    Route route_vhc_1_bkp = solution.routes_delivery_[vhc_1];

    if (is_b_mov) {
        while (solution.routes_delivery_[vhc_1].visited_nodes_[pos_1 + 1] != 0) {
            solution.routes_delivery_[vhc_1].RemoveVisitedNodeFromPos(pos_1 + 1);
        }
        for (int i = pos_2; i > 0; --i) {
            new_reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs);
            solution.schedule_manager_.pic_del_[solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs].second = vhc_1;
            solution.routes_delivery_[vhc_1].InsertVisitedNode(solution.routes_delivery_[vhc_1].last_insertable_pos_, solution.routes_delivery_[vhc_2].visited_nodes_[i]);
        }

        Route route_vhc_2_bkp = solution.routes_delivery_[vhc_2];
        solution.routes_delivery_[vhc_2].Clear();

        for (int i = route_vhc_1_bkp.num_nodes_; i >= pos_1 + 1; --i) {
            new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i] - num_reqs);
            solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i] - num_reqs].second = vhc_2;
            solution.routes_delivery_[vhc_2].InsertVisitedNode(solution.routes_delivery_[vhc_2].last_insertable_pos_, route_vhc_1_bkp.visited_nodes_[i]);
        }
        for (int i = pos_2 + 1; i <= route_vhc_2_bkp.num_nodes_; ++i) {
            solution.routes_delivery_[vhc_2].InsertVisitedNode(solution.routes_delivery_[vhc_2].last_insertable_pos_, route_vhc_2_bkp.visited_nodes_[i]);
        }

    } else if (!change_vhcs_a_mov) {
        while (solution.routes_delivery_[vhc_1].visited_nodes_[pos_1 + 1] != 0) {
            solution.routes_delivery_[vhc_1].RemoveVisitedNodeFromPos(pos_1 + 1);
        }
        for (int i = pos_2; i <= solution.routes_delivery_[vhc_2].num_nodes_; ++i) {
            new_reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs);
            solution.schedule_manager_.pic_del_[solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs].second = vhc_1;
            solution.routes_delivery_[vhc_1].InsertVisitedNode(solution.routes_delivery_[vhc_1].last_insertable_pos_, solution.routes_delivery_[vhc_2].visited_nodes_[i]);
        }

        while (solution.routes_delivery_[vhc_2].visited_nodes_[pos_2] != 0) {
            solution.routes_delivery_[vhc_2].RemoveVisitedNodeFromPos(pos_2);
        }
        for (int i = pos_1 + 1; i <= route_vhc_1_bkp.num_nodes_; ++i) {
            new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i] - num_reqs);
            solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i] - num_reqs].second = vhc_2;
            solution.routes_delivery_[vhc_2].InsertVisitedNode(solution.routes_delivery_[vhc_2].last_insertable_pos_, route_vhc_1_bkp.visited_nodes_[i]);
        }
    } else {
        while (solution.routes_delivery_[vhc_1].visited_nodes_[1] != route_vhc_1_bkp.visited_nodes_[pos_1 + 1]) {
            solution.routes_delivery_[vhc_1].RemoveVisitedNodeFromPos(1);
        }
        for (int i = 1; i < pos_2; ++i) {
            new_reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs);
            solution.schedule_manager_.pic_del_[solution.routes_delivery_[vhc_2].visited_nodes_[i] - num_reqs].second = vhc_1;
            solution.routes_delivery_[vhc_1].InsertVisitedNode(i, solution.routes_delivery_[vhc_2].visited_nodes_[i]);
        }

        int stop_node = solution.routes_delivery_[vhc_2].visited_nodes_[pos_2];
        while (solution.routes_delivery_[vhc_2].visited_nodes_[1] != stop_node) {
            solution.routes_delivery_[vhc_2].RemoveVisitedNodeFromPos(1);
        }
        for (int i = 1; i <= pos_1; ++i) {
            new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i] - num_reqs);
            solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i] - num_reqs].second = vhc_2;
            solution.routes_delivery_[vhc_2].InsertVisitedNode(i, route_vhc_1_bkp.visited_nodes_[i]);
        }
    }

    Schedules & schedules = solution.schedule_manager_.schedules_;

    // unloading updates
    for (auto & vhc_unl : vhc_unload_order) {
        schedules[vhc_unl.first].unloaded_reqs_.clear();
        for (int r : vhc_unl.second) {
            schedules[vhc_unl.first].unloaded_reqs_.push_back(UnloadUnit(r));
        }
    }

    // reloading updates
    for (uint i = 0; i < schedules[vhc_1].reloaded_reqs_.size(); ++i) {
        if (new_reqs_in_vhc_2.find(schedules[vhc_1].reloaded_reqs_[i].ind_) != new_reqs_in_vhc_2.end()) {
            schedules[vhc_1].reloaded_reqs_.erase(schedules[vhc_1].reloaded_reqs_.begin() + i);
            --i;
        }
    }
    for (auto it = new_reqs_in_vhc_1.begin(); it != new_reqs_in_vhc_1.end(); ++it) {
        if (solution.schedule_manager_.pic_del_[*it].first != vhc_1) {
            schedules[vhc_1].reloaded_reqs_.push_back(ReloadUnit(*it));
        }
    }

    for (uint i = 0; i < schedules[vhc_2].reloaded_reqs_.size(); ++i) {
        if (new_reqs_in_vhc_1.find(schedules[vhc_2].reloaded_reqs_[i].ind_) != new_reqs_in_vhc_1.end()) {
            schedules[vhc_2].reloaded_reqs_.erase(schedules[vhc_2].reloaded_reqs_.begin() + i);
            --i;
        }
    }
    for (auto it = new_reqs_in_vhc_2.begin(); it != new_reqs_in_vhc_2.end(); ++it) {
        if (solution.schedule_manager_.pic_del_[*it].first != vhc_2) {
            schedules[vhc_2].reloaded_reqs_.push_back(ReloadUnit(*it));
        }
    }

    // update times
    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);

    int vhc;
    bool rel_not_found;
    for (auto & it : vhc_unload_order) {
        vhc = it.first;
        aux_start_unload = schedules[vhc].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[vhc].unloaded_reqs_.size(); ++i) {
            schedules[vhc].unloaded_reqs_[i].start_time_ = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[i].ind_);
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second);

            rel_not_found = true;
            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                    == schedules[vhc].unloaded_reqs_[i].ind_) {
                    
                    schedules[solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                            schedules[vhc].unloaded_reqs_[i].start_time_
                            + Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[i].ind_)
                            + Instance::instance()->between_docks_time(vhc, solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second);
                    rel_not_found = false;
                    break;
                }
            }

            if (rel_not_found) {
                schedules[solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second].reloaded_reqs_.push_back(
                        ReloadUnit(schedules[vhc].unloaded_reqs_[i].ind_, schedules[vhc].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(vhc, solution.schedule_manager_.pic_del_[schedules[vhc].unloaded_reqs_[i].ind_].second)));
            }
        }
        if (schedules[vhc].unloaded_reqs_.empty()) {
            schedules[vhc].unload_end_ = schedules[vhc].start_time_;
        } else {
            schedules[vhc].unload_end_ = aux_start_unload;
        }
    }

    for (int vhc : {vhc_1, vhc_2}) {
        for (uint i = 0; i < schedules[vhc].reloaded_reqs_.size(); ++i) {
            if (cmp(schedules[vhc].reloaded_reqs_[i].availability_time_, 0.0) == 0) {
                for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[vhc].reloaded_reqs_[i].ind_].first].unloaded_reqs_.size(); ++j) {
                    if (schedules[solution.schedule_manager_.pic_del_[schedules[vhc].reloaded_reqs_[i].ind_].first].unloaded_reqs_[j].ind_ == schedules[vhc].reloaded_reqs_[i].ind_) {
                        schedules[vhc].reloaded_reqs_[i].availability_time_ = schedules[solution.schedule_manager_.pic_del_[schedules[vhc].reloaded_reqs_[i].ind_].first].unloaded_reqs_[j].start_time_
                                                                              + Instance::instance()->unloading_time(schedules[solution.schedule_manager_.pic_del_[schedules[vhc].reloaded_reqs_[i].ind_].first].unloaded_reqs_[j].ind_)
                                                                              + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[schedules[vhc].reloaded_reqs_[i].ind_].first, vhc);
                    }
                }
            }
        }
    }

    double infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

            schedules[*it].completion_time_ = schedules[*it].reloaded_reqs_.back().start_time_ + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_.back().ind_);
        } else {
            schedules[*it].completion_time_ = schedules[*it].unload_end_;
        }
    }

    solution.cost_ = 0.0;
    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution.cost_ += solution.routes_delivery_[k].cost_ + schedules[k].completion_time_;
    }
}

bool LocalSearches::ER_Exchange_in_best_pos(Solution & solution, LS_strat search_strat) {
    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    bool improve, general_improvement, is_pickup, best_is_pickup;
    int orig_pos, k1, best_pos_1 = -1, best_pos_2 = -1, best_vhc_1 = -1, best_vhc_2 = -1, free_space_k1, orig_demand, selected_req;
    double best_cost, temp_cost, current_cost;
    int final_pos_for_n1, final_pos_for_n2, best_fpfn1 = -1, best_fpfn2 = -1;
    int pos_r1, pos_r2, best_pos_r1 = -1, best_pos_r2 = -1;
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
            k1 = is_pickup ? solution.schedule_manager_.pic_del_[selected_req].first : solution.schedule_manager_.pic_del_[selected_req].second;
            orig_pos = find(routes[k1].visited_nodes_.begin(), routes[k1].visited_nodes_.end(), *selected_node) - routes[k1].visited_nodes_.begin();
            orig_demand = Instance::instance()->node(*selected_node).demand();
            free_space_k1 = vhc_capacity - routes[k1].load_ + orig_demand;

            temp_cost = solution.cost_;

            for (int k2 = 0; k2 < num_vehicles; ++k2) {
                if (k1 != k2) {
                    for (int j = Route::first_insertable_pos_; j <= routes[k2].num_nodes_; ++j) {
                        if (!nodes_already_selected[routes[k2].visited_nodes_[j]]
                            && free_space_k1 >= Instance::instance()->node(routes[k2].visited_nodes_[j]).demand()
                            && vhc_capacity - routes[k2].load_ + Instance::instance()->node(routes[k2].visited_nodes_[j]).demand() >= orig_demand) {

                            current_cost = temp_cost + (is_pickup ? ER_exchange_in_best_pos_pickup_delta_cost(solution, k1, orig_pos, k2, j, final_pos_for_n1, final_pos_for_n2, pos_r1, pos_r2)
                                                        : ER_exchange_in_best_pos_delivery_delta_cost(solution, k1, orig_pos, k2, j, final_pos_for_n1, final_pos_for_n2, pos_r1, pos_r2));

                            if (cmp(current_cost, best_cost) < 0) {
                                best_cost = current_cost;
                                best_pos_1 = orig_pos;
                                best_pos_2 = j;
                                best_vhc_1 = k1;
                                best_vhc_2 = k2;
                                best_fpfn1 = final_pos_for_n1;
                                best_fpfn2 = final_pos_for_n2;
                                best_pos_r1 = pos_r1;
                                best_pos_r2 = pos_r2;
                                best_is_pickup = is_pickup;
                                improve = true;
                            }
                        }
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;
                if (best_is_pickup) {
                    ER_move_exchange_in_best_pos_pickup_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_fpfn1, best_fpfn2, best_pos_r1, best_pos_r2);
                } else {
                    ER_move_exchange_in_best_pos_delivery_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_fpfn1, best_fpfn2, best_pos_r1, best_pos_r2);
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
                ER_move_exchange_in_best_pos_pickup_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_fpfn1, best_fpfn2, best_pos_r1, best_pos_r2);
            } else {
                ER_move_exchange_in_best_pos_delivery_routes(solution, best_vhc_1, best_pos_1, best_vhc_2, best_pos_2, best_fpfn1, best_fpfn2, best_pos_r1, best_pos_r2);
            }

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }
    }

    return general_improvement;
}

double LocalSearches::ER_exchange_in_best_pos_pickup_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, int & final_pos_for_n1, int & final_pos_for_n2, int & unl_pos_r1, int & unl_pos_r2) {
    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    int node_1 = solution.routes_pickup_[vhc_1].visited_nodes_[pos_1];
    int node_2 = solution.routes_pickup_[vhc_2].visited_nodes_[pos_2];
    double vhc_1_route_cost, vhc_2_route_cost;
    double vhc_1_unload_end, vhc_2_unload_end;
    double best_cost, temp_cost, curr_cost;

    // delta cost on routes
    best_cost = 1000.0 * solution.routes_pickup_[vhc_1].cost_;
    temp_cost = solution.routes_pickup_[vhc_1].cost_
                - Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_1 - 1], node_1)
                - Instance::instance()->EdgeCost(node_1, solution.routes_pickup_[vhc_1].visited_nodes_[pos_1 + 1])
                + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_1 - 1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_1 + 1]);

    for (int i = 1; i <= solution.routes_pickup_[vhc_1].last_insertable_pos_; ++i) {
        if (i == pos_1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i - 1], node_2)
                        + Instance::instance()->EdgeCost(node_2, solution.routes_pickup_[vhc_1].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i - 1], solution.routes_pickup_[vhc_1].visited_nodes_[i + 1]);
        } else if (i != pos_1 + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i - 1], node_2)
                        + Instance::instance()->EdgeCost(node_2, solution.routes_pickup_[vhc_1].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i - 1], solution.routes_pickup_[vhc_1].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_1) {
                final_pos_for_n2 = i-1;
            } else {
                final_pos_for_n2 = i;
            }
        }
    }

    vhc_1_route_cost = best_cost;

    best_cost = 1000.0 * solution.routes_pickup_[vhc_2].cost_;
    temp_cost = solution.routes_pickup_[vhc_2].cost_
                - Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_2 - 1], node_2)
                - Instance::instance()->EdgeCost(node_2, solution.routes_pickup_[vhc_2].visited_nodes_[pos_2 + 1])
                + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_2 - 1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_2 + 1]);

    for (int i = 1; i <= solution.routes_pickup_[vhc_2].last_insertable_pos_; ++i) {
        if (i == pos_2) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i - 1], node_1)
                        + Instance::instance()->EdgeCost(node_1, solution.routes_pickup_[vhc_2].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i - 1], solution.routes_pickup_[vhc_2].visited_nodes_[i + 1]);
        } else if (i != pos_2 + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i - 1], node_1)
                        + Instance::instance()->EdgeCost(node_1, solution.routes_pickup_[vhc_2].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[i - 1], solution.routes_pickup_[vhc_2].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_2) {
                final_pos_for_n1 = i-1;
            } else {
                final_pos_for_n1 = i;
            }
        }
    }

    vhc_2_route_cost = best_cost;

    // unloading updates
    double aux_start_unload;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    // on vhc_1
    aux_start_unload = vhc_1_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
        if (schedules[vhc_1].unloaded_reqs_[i].ind_ != node_1) {
            req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
        }
    }

    if (solution.schedule_manager_.pic_del_[node_2].second != vhc_1) {
        req_start_unload[node_2] = aux_start_unload;
        vhc_1_unload_end = aux_start_unload + Instance::instance()->unloading_time(node_2);
    } else {
        req_start_unload[node_2] = 0.0;
        if (schedules[vhc_1].unloaded_reqs_.empty()
            || (schedules[vhc_1].unloaded_reqs_.size() == 1
                && schedules[vhc_1].unloaded_reqs_[0].ind_ ==  node_1)) {
            vhc_1_unload_end = vhc_1_route_cost;
        } else {
            vhc_1_unload_end = aux_start_unload;
        }
    }

    // on vhc_2
    aux_start_unload = vhc_2_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
        if (schedules[vhc_2].unloaded_reqs_[i].ind_ != node_2) {
            req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
        }
    }

    if (solution.schedule_manager_.pic_del_[node_1].second != vhc_2) {
        req_start_unload[node_1] = aux_start_unload;
        vhc_2_unload_end = aux_start_unload + Instance::instance()->unloading_time(node_1);
    } else {
        req_start_unload[node_1] = 0.0;
        if (schedules[vhc_2].unloaded_reqs_.empty()
            || (schedules[vhc_2].unloaded_reqs_.size() == 1
                && schedules[vhc_2].unloaded_reqs_[0].ind_ == node_2)) {
            vhc_2_unload_end = vhc_2_route_cost;
        } else {
            vhc_2_unload_end = aux_start_unload;
        }
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_unload_end, infeas_st_av_size;

    vector<double> start_reload(num_reqs + 1, -1.0), ct_delta(num_vhcs, 0.0);

    for (int k = 0; k < num_vhcs; ++k) {
        for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
            start_reload[schedules[k].reloaded_reqs_[i].ind_] = schedules[k].reloaded_reqs_[i].start_time_;
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (rs[i].ind_ != node_1 && rs[i].ind_ != node_2) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
                } else if (rs[i].ind_ == node_1) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
                    }
                } else if (rs[i].ind_ == node_2) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_1) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
                    }
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            if (solution.schedule_manager_.pic_del_[node_1].first == solution.schedule_manager_.pic_del_[node_1].second) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = node_1;
                rs[rs.size() - 1].availability_time_ = req_start_unload[node_1] + Instance::instance()->unloading_time(node_1) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            if (solution.schedule_manager_.pic_del_[node_2].first == solution.schedule_manager_.pic_del_[node_2].second) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = node_2;
                rs[rs.size() - 1].availability_time_ = req_start_unload[node_2] + Instance::instance()->unloading_time(node_2) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
            }
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
                    start_reload[rs[i].ind_] = rs[i].start_time_;
                }
            }

            ct_delta[*rld_vhc] = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
        } else {
            ct_delta[*rld_vhc] = curr_unload_end - schedules[*rld_vhc].completion_time_;
        }
    }

    impacted_reloads.clear();

    if (solution.schedule_manager_.pic_del_[node_1].second != vhc_2) {
        bool after_node_2 = false;
        if (vhc_2 != solution.schedule_manager_.pic_del_[node_2].second) {
            after_node_2 = true;
        }
        unl_pos_r1 = schedules[vhc_2].unloaded_reqs_.size() - (after_node_2 ? 1 : 0);
        for (int i = schedules[vhc_2].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (schedules[vhc_2].unloaded_reqs_[i].ind_ != node_2
                && cmp(Instance::instance()->unloading_time(node_1),
                       start_reload[schedules[vhc_2].unloaded_reqs_[i].ind_]
                       - (req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_]
                          + Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_)
                          + Instance::instance()->between_docks_time(vhc_2, solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second))) <= 0) {
                req_start_unload[node_1] -= Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(node_1);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[node_1].second);
                unl_pos_r1 = i - (after_node_2 ? 1 : 0);
            } else if (schedules[vhc_2].unloaded_reqs_[i].ind_ == node_2) {
                after_node_2 = false;
            } else {
                break;
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[node_2].second != vhc_1) {
        bool after_node_1 = false;
        if (vhc_1 != solution.schedule_manager_.pic_del_[node_1].second) {
            after_node_1= true;
        }
        unl_pos_r2 = schedules[vhc_1].unloaded_reqs_.size() - (after_node_1 ? 1 : 0);
        for (int i = schedules[vhc_1].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (schedules[vhc_1].unloaded_reqs_[i].ind_ != node_1
                && cmp(Instance::instance()->unloading_time(node_2),
                       start_reload[schedules[vhc_1].unloaded_reqs_[i].ind_]
                       - (req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_]
                          + Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_)
                          + Instance::instance()->between_docks_time(vhc_1, solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second))) <= 0) {
                req_start_unload[node_2] -= Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(node_2);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[node_2].second);
                unl_pos_r2 = i - (after_node_1 ? 1 : 0);
            } else if (schedules[vhc_1].unloaded_reqs_[i].ind_ == node_1) {
                after_node_1 = false;
            } else {
                break;
            }
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (rs[i].ind_ != node_1 && rs[i].ind_ != node_2) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
                } else if (rs[i].ind_ == node_1) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
                    }
                } else if (rs[i].ind_ == node_2) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_1) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
                    }
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            if (solution.schedule_manager_.pic_del_[node_1].first == solution.schedule_manager_.pic_del_[node_1].second) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = node_1;
                rs[rs.size() - 1].availability_time_ = req_start_unload[node_1] + Instance::instance()->unloading_time(node_1) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            if (solution.schedule_manager_.pic_del_[node_2].first == solution.schedule_manager_.pic_del_[node_2].second) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = node_2;
                rs[rs.size() - 1].availability_time_ = req_start_unload[node_2] + Instance::instance()->unloading_time(node_2) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
            }
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

            ct_delta[*rld_vhc] = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
        } else {
            ct_delta[*rld_vhc] = curr_unload_end - schedules[*rld_vhc].completion_time_;
        }
    }

    for (int k = 0; k < num_vhcs; ++k) {
        delta_cost += ct_delta[k];
    }

    return delta_cost;
}

double LocalSearches::ER_exchange_in_best_pos_delivery_delta_cost(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, int & final_pos_for_n1, int & final_pos_for_n2, int & unl_pos_r1, int & unl_pos_r2) {
    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    int node_1 = solution.routes_delivery_[vhc_1].visited_nodes_[pos_1];
    int node_2 = solution.routes_delivery_[vhc_2].visited_nodes_[pos_2];
    int req_1 = node_1 - num_reqs;
    int req_2 = node_2 - num_reqs;
    double vhc_1_route_cost, vhc_2_route_cost;
    double vhc_1_diff_route_cost, vhc_2_diff_route_cost;
    double vhc_1_unload_end, vhc_2_unload_end;
    double best_cost, temp_cost, curr_cost;

    unl_pos_r1 = unl_pos_r2 = -1;

    // delta cost on routes
    best_cost = 1000.0 * solution.routes_delivery_[vhc_1].cost_;
    temp_cost = solution.routes_delivery_[vhc_1].cost_
                - Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_1 - 1], node_1)
                - Instance::instance()->EdgeCost(node_1, solution.routes_delivery_[vhc_1].visited_nodes_[pos_1 + 1])
                + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_1 - 1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_1 + 1]);

    for (int i = 1; i <= solution.routes_delivery_[vhc_1].last_insertable_pos_; ++i) {
        if (i == pos_1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i - 1], node_2)
                        + Instance::instance()->EdgeCost(node_2, solution.routes_delivery_[vhc_1].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i - 1], solution.routes_delivery_[vhc_1].visited_nodes_[i + 1]);
        } else if (i != pos_1 + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i - 1], node_2)
                        + Instance::instance()->EdgeCost(node_2, solution.routes_delivery_[vhc_1].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i - 1], solution.routes_delivery_[vhc_1].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_1) {
                final_pos_for_n2 = i-1;
            } else {
                final_pos_for_n2 = i;
            }
        }
    }

    vhc_1_route_cost = best_cost;

    best_cost = 1000.0 * solution.routes_delivery_[vhc_2].cost_;
    temp_cost = solution.routes_delivery_[vhc_2].cost_
                - Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_2 - 1], node_2)
                - Instance::instance()->EdgeCost(node_2, solution.routes_delivery_[vhc_2].visited_nodes_[pos_2 + 1])
                + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_2 - 1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_2 + 1]);

    for (int i = 1; i <= solution.routes_delivery_[vhc_2].last_insertable_pos_; ++i) {
        if (i == pos_2) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i - 1], node_1)
                        + Instance::instance()->EdgeCost(node_1, solution.routes_delivery_[vhc_2].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i - 1], solution.routes_delivery_[vhc_2].visited_nodes_[i + 1]);
        } else if (i != pos_2 + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i - 1], node_1)
                        + Instance::instance()->EdgeCost(node_1, solution.routes_delivery_[vhc_2].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[i - 1], solution.routes_delivery_[vhc_2].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_2) {
                final_pos_for_n1 = i-1;
            } else {
                final_pos_for_n1 = i;
            }
        }
    }

    vhc_2_route_cost = best_cost;

    vhc_1_diff_route_cost = vhc_1_route_cost - solution.routes_delivery_[vhc_1].cost_;
    vhc_2_diff_route_cost = vhc_2_route_cost - solution.routes_delivery_[vhc_2].cost_;

    delta_cost = vhc_1_diff_route_cost + vhc_2_diff_route_cost;

    Schedules & schedules = solution.schedule_manager_.schedules_;

    vhc_1_unload_end = schedules[vhc_1].unload_end_;
    vhc_2_unload_end = schedules[vhc_2].unload_end_;

    req_start_unload[req_1] = req_start_unload[req_2] = 0.0;

    if (solution.schedule_manager_.pic_del_[req_1].first != vhc_1) {
        for (uint i = 0; i < schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.size(); ++i) {
            if (schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_[i].ind_ == req_1) {
                req_start_unload[req_1] = schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_[i].start_time_;
                break;
            }
        }
    }
    if (solution.schedule_manager_.pic_del_[req_2].first != vhc_2) {
        for (uint i = 0; i < schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_.size(); ++i) {
            if (schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_[i].ind_ == req_2) {
                req_start_unload[req_2] = schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_[i].start_time_;
                break;
            }
        }
    }

    // unloading updates
    double aux_start_unload = -1.0;
    if (solution.schedule_manager_.pic_del_[req_2].first == vhc_1) { // doesn't unload anymore
        bool after_node_2 = false;
        for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
            if (schedules[vhc_1].unloaded_reqs_[i].ind_ == req_2) {
                aux_start_unload = schedules[vhc_1].unloaded_reqs_[i].start_time_;
                after_node_2 = true;
            } else if (after_node_2) {
                req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] = aux_start_unload;
                aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
            }
        }
        if (schedules[vhc_1].unloaded_reqs_.size() == 1) {
            vhc_1_unload_end = schedules[vhc_1].start_time_;
        } else {
            vhc_1_unload_end = aux_start_unload;
        }
    }

    if (solution.schedule_manager_.pic_del_[req_1].first == vhc_2) { // doesn't unload anymore
        bool after_node_1 = false;
        for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
            if (schedules[vhc_2].unloaded_reqs_[i].ind_ == req_1) {
                aux_start_unload = schedules[vhc_2].unloaded_reqs_[i].start_time_;
                after_node_1 = true;
            } else if (after_node_1) {
                req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] = aux_start_unload;
                aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
            }
        }
        if (schedules[vhc_2].unloaded_reqs_.size() == 1) {
            vhc_2_unload_end = schedules[vhc_2].start_time_;
        } else {
            vhc_2_unload_end = aux_start_unload;
        }
    }

    if (solution.schedule_manager_.pic_del_[req_1].first == solution.schedule_manager_.pic_del_[req_1].second) { // starts unloading
        if (schedules[vhc_1].unloaded_reqs_.empty()
            || (schedules[vhc_1].unloaded_reqs_.size() == 1
                && schedules[vhc_1].unloaded_reqs_[0].ind_ == req_2)) {
            req_start_unload[req_1] = schedules[vhc_1].start_time_ + Instance::instance()->unload_preparation_time();
        } else {
            req_start_unload[req_1] = vhc_1_unload_end;
        }
        vhc_1_unload_end = req_start_unload[req_1] + Instance::instance()->unloading_time(req_1);
    }

    if (solution.schedule_manager_.pic_del_[req_2].first == solution.schedule_manager_.pic_del_[req_2].second) { // starts unloading
        if (schedules[vhc_2].unloaded_reqs_.empty()
            || (schedules[vhc_2].unloaded_reqs_.size() == 1
                && schedules[vhc_2].unloaded_reqs_[0].ind_ == req_1)) {
            req_start_unload[req_2] = schedules[vhc_2].start_time_ + Instance::instance()->unload_preparation_time();
        } else {
            req_start_unload[req_2] = vhc_2_unload_end;
        }
        vhc_2_unload_end = req_start_unload[req_2] + Instance::instance()->unloading_time(req_2);
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double curr_delta, curr_unload_end, infeas_st_av_size;

    vector<double> start_reload(num_reqs + 1, -1.0), ct_delta(num_vhcs, 0.0);

    for (int k = 0; k < num_vhcs; ++k) {
        for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
            start_reload[schedules[k].reloaded_reqs_[i].ind_] = schedules[k].reloaded_reqs_[i].start_time_;
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (rs[i].ind_ != req_1 && rs[i].ind_ != req_2) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                } else {
                    rs.erase(rs.begin() + i);
                    --i;
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            if (solution.schedule_manager_.pic_del_[req_2].first != vhc_1) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = req_2;
                rs[rs.size() - 1].availability_time_ = req_start_unload[req_2] + Instance::instance()->unloading_time(req_2) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[req_2].first, *rld_vhc);
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            if (solution.schedule_manager_.pic_del_[req_1].first != vhc_2) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = req_1;
                rs[rs.size() - 1].availability_time_ = req_start_unload[req_1] + Instance::instance()->unloading_time(req_1) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[req_1].first, *rld_vhc);
            }
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
                    start_reload[rs[i].ind_] = rs[i].start_time_;
                }
            }

            curr_delta = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*rld_vhc].completion_time_;
            ct_delta[*rld_vhc] = curr_delta;
        } else {
            curr_delta = curr_unload_end - schedules[*rld_vhc].completion_time_;
            ct_delta[*rld_vhc] = curr_delta;
        }
    }

    impacted_reloads.clear();

    set<int> vhcs_to_update;
    if (solution.schedule_manager_.pic_del_[req_1].first != vhc_2) {
        vhcs_to_update.insert(solution.schedule_manager_.pic_del_[req_1].first);
    }
    if (solution.schedule_manager_.pic_del_[req_2].first != vhc_1) {
        vhcs_to_update.insert(solution.schedule_manager_.pic_del_[req_2].first);
    }

    for (auto v = vhcs_to_update.begin(); v != vhcs_to_update.end(); ++v) {
        aux_start_unload = schedules[*v].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[*v].unloaded_reqs_.size(); ++i) {
            if ((schedules[*v].unloaded_reqs_[i].ind_ == req_1 || schedules[*v].unloaded_reqs_[i].ind_ == req_2) && (*v == vhc_1 || *v == vhc_2)) {
                continue;
            } else {
                req_start_unload[schedules[*v].unloaded_reqs_[i].ind_] = aux_start_unload;
                aux_start_unload += Instance::instance()->unloading_time(schedules[*v].unloaded_reqs_[i].ind_);
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[req_1].first == vhc_1) {
        bool after_req_2 = false;
        if (solution.schedule_manager_.pic_del_[req_2].first == vhc_1) {
            after_req_2 = true;
        }
        unl_pos_r1 = schedules[vhc_1].unloaded_reqs_.size() - (after_req_2 ? 1 : 0);
        for (int i = schedules[vhc_1].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (schedules[vhc_1].unloaded_reqs_[i].ind_ != req_2
                && cmp(Instance::instance()->unloading_time(req_1),
                       start_reload[schedules[vhc_1].unloaded_reqs_[i].ind_]
                       - (req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_]
                          + Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_)
                          + Instance::instance()->between_docks_time(vhc_1, solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second))) <= 0) {
                req_start_unload[req_1] -= Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[vhc_1].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(req_1);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(vhc_2);
                unl_pos_r1 = i - (after_req_2 ? 1 : 0);
            } else if (schedules[vhc_1].unloaded_reqs_[i].ind_ == req_2) {
                after_req_2 = false;
            } else {
                break;
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[req_2].first == vhc_2) {
        bool after_req_1 = false;
        if (solution.schedule_manager_.pic_del_[req_1].first == vhc_2) {
            after_req_1 = true;
        }
        unl_pos_r2 = schedules[vhc_2].unloaded_reqs_.size() - (after_req_1 ? 1 : 0);
        for (int i = schedules[vhc_2].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (schedules[vhc_2].unloaded_reqs_[i].ind_ != req_1
                && cmp(Instance::instance()->unloading_time(req_2),
                       start_reload[schedules[vhc_2].unloaded_reqs_[i].ind_]
                       - (req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_]
                          + Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_)
                          + Instance::instance()->between_docks_time(vhc_2, solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second))) <= 0) {
                req_start_unload[req_2] -= Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[vhc_2].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(req_2);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(vhc_1);
                unl_pos_r2 = i - (after_req_1 ? 1 : 0);
            } else if (schedules[vhc_2].unloaded_reqs_[i].ind_ == req_1) {
                after_req_1 = false;
            } else {
                break;
            }
        }
    }

    int orig_pos_r1;
    if (solution.schedule_manager_.pic_del_[req_1].first != vhc_1 && solution.schedule_manager_.pic_del_[req_1].first != vhc_2) { // pic only req_1 on vhc_3
        int unload_vhc = solution.schedule_manager_.pic_del_[req_1].first;
        if (cmp(schedules[vhc_1].unload_end_, vhc_2_unload_end) < 0) {
            double req_start_reload = 0.0;
            bool after_req_1 = false;
            double vhc_2_reload_start = vhc_2_unload_end + Instance::instance()->reload_preparation_time();
            for (uint i = 0; i < schedules[unload_vhc].unloaded_reqs_.size(); ++i) {
                if (schedules[unload_vhc].unloaded_reqs_[i].ind_ == req_1) {
                    orig_pos_r1 = unl_pos_r1 = i;
                    req_start_reload = req_start_unload[req_1]
                                       + Instance::instance()->unloading_time(req_1)
                                       + Instance::instance()->between_docks_time(unload_vhc, vhc_2);
                    after_req_1 = true;
                } else if (after_req_1) {
                    if (cmp(req_start_reload + Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_), vhc_2_reload_start) <= 0) {
                        req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_] -= Instance::instance()->unloading_time(req_1);
                        req_start_unload[req_1] += Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                        if (schedules[unload_vhc].unloaded_reqs_[i].ind_ != req_2) {
                            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[unload_vhc].unloaded_reqs_[i].ind_].second);
                        } else {
                            impacted_reloads.insert(vhc_1);
                        }
                        impacted_reloads.insert(vhc_2);
                        req_start_reload += Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                        unl_pos_r1 = i;
                    } else {
                        break;
                    }
                }
            }
        } else {
            bool before_req_1 = false;
            for (int i = schedules[unload_vhc].unloaded_reqs_.size() - 1; i >= 0; --i) {
                if (schedules[unload_vhc].unloaded_reqs_[i].ind_ == req_1) {
                    orig_pos_r1 = unl_pos_r1 = i;
                    before_req_1 = true;
                } else if (before_req_1) {
                    if (cmp(Instance::instance()->unloading_time(req_1),
                            start_reload[schedules[unload_vhc].unloaded_reqs_[i].ind_]
                            - (req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_]
                               + Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_)
                               + Instance::instance()->between_docks_time(unload_vhc, (schedules[unload_vhc].unloaded_reqs_[i].ind_ == req_2 ? vhc_1 : solution.schedule_manager_.pic_del_[schedules[unload_vhc].unloaded_reqs_[i].ind_].second)))) <= 0) {
                        req_start_unload[req_1] = req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_];
                        req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(req_1);
                        if (schedules[unload_vhc].unloaded_reqs_[i].ind_ != req_2) {
                            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[unload_vhc].unloaded_reqs_[i].ind_].second);
                        } else {
                            impacted_reloads.insert(vhc_1);
                        }
                        impacted_reloads.insert(vhc_2);
                        unl_pos_r1 = i;
                    } else {
                        break;
                    }
                }
            }
        }

        if (solution.schedule_manager_.pic_del_[req_1].first == solution.schedule_manager_.pic_del_[req_2].first) { // vhc_3 pic req_2 also
            vector<int> aux_unloaded_reqs;
            for (int i = 0; i < static_cast<int>(schedules[unload_vhc].unloaded_reqs_.size()); ++i) {
                if (i == unl_pos_r1) {
                    if (unl_pos_r1 > orig_pos_r1) {
                        aux_unloaded_reqs.push_back(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                        aux_unloaded_reqs.push_back(req_1);
                    } else if (unl_pos_r1 < orig_pos_r1) {
                        aux_unloaded_reqs.push_back(req_1);
                        aux_unloaded_reqs.push_back(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                    } else {
                        aux_unloaded_reqs.push_back(req_1);
                    }
                } else if (schedules[unload_vhc].unloaded_reqs_[i].ind_ != req_1) {
                    aux_unloaded_reqs.push_back(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                }
            }

            if (cmp(schedules[vhc_2].unload_end_, vhc_1_unload_end) < 0) {
                double req_start_reload = 0.0;
                bool after_req_2 = false;
                double vhc_1_reload_start = vhc_1_unload_end + Instance::instance()->reload_preparation_time();
                for (uint i = 0; i < aux_unloaded_reqs.size(); ++i) {
                    if (aux_unloaded_reqs[i] == req_2) {
                        unl_pos_r2 = i;
                        req_start_reload = req_start_unload[req_2]
                                           + Instance::instance()->unloading_time(req_2)
                                           + Instance::instance()->between_docks_time(unload_vhc, vhc_1);
                        after_req_2 = true;
                    } else if (after_req_2) {
                        if (cmp(req_start_reload + Instance::instance()->unloading_time(aux_unloaded_reqs[i]), vhc_1_reload_start) <= 0) {
                            req_start_unload[aux_unloaded_reqs[i]] -= Instance::instance()->unloading_time(req_2);
                            req_start_unload[req_2] += Instance::instance()->unloading_time(aux_unloaded_reqs[i]);
                            if (aux_unloaded_reqs[i] != req_1) {
                                impacted_reloads.insert(solution.schedule_manager_.pic_del_[aux_unloaded_reqs[i]].second);
                            } else {
                                impacted_reloads.insert(vhc_2);
                                --unl_pos_r1;
                            }
                            impacted_reloads.insert(vhc_1);
                            req_start_reload += Instance::instance()->unloading_time(aux_unloaded_reqs[i]);
                            unl_pos_r2 = i;
                        } else {
                            break;
                        }
                    }
                }

            } else {
                bool before_req_2 = false;
                for (int i = aux_unloaded_reqs.size() - 1; i >= 0; --i) {
                    if (aux_unloaded_reqs[i] == req_2) {
                        unl_pos_r2 = i;
                        before_req_2 = true;
                    } else if (before_req_2) {
                        if (cmp(Instance::instance()->unloading_time(req_2),
                                start_reload[aux_unloaded_reqs[i]]
                                - (req_start_unload[aux_unloaded_reqs[i]]
                                   + Instance::instance()->unloading_time(aux_unloaded_reqs[i])
                                   + Instance::instance()->between_docks_time(unload_vhc, (aux_unloaded_reqs[i] == req_1 ? vhc_2 : solution.schedule_manager_.pic_del_[aux_unloaded_reqs[i]].second)))) <= 0) {
                            req_start_unload[req_2] = req_start_unload[aux_unloaded_reqs[i]];
                            req_start_unload[aux_unloaded_reqs[i]] += Instance::instance()->unloading_time(req_2);
                            if (aux_unloaded_reqs[i] != req_1) {
                                impacted_reloads.insert(solution.schedule_manager_.pic_del_[aux_unloaded_reqs[i]].second);
                            } else {
                                impacted_reloads.insert(vhc_2);
                                ++unl_pos_r1;
                            }
                            impacted_reloads.insert(vhc_1);
                            unl_pos_r2 = i;
                        } else {
                            break;
                        }
                    }
                }
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[req_2].first != vhc_1 && solution.schedule_manager_.pic_del_[req_2].first != vhc_2) { // pic only req_2 on vhc_3
        if (solution.schedule_manager_.pic_del_[req_1].first != solution.schedule_manager_.pic_del_[req_2].first) {
            int unload_vhc = solution.schedule_manager_.pic_del_[req_2].first;
            if (cmp(schedules[vhc_2].unload_end_, vhc_1_unload_end) < 0) {
                double req_start_reload = 0.0;
                bool after_req_2 = false;
                double vhc_1_reload_start = vhc_1_unload_end + Instance::instance()->reload_preparation_time();
                for (uint i = 0; i < schedules[unload_vhc].unloaded_reqs_.size(); ++i) {
                    if (schedules[unload_vhc].unloaded_reqs_[i].ind_ == req_2) {
                        unl_pos_r2 = i;
                        req_start_reload = req_start_unload[req_2]
                                           + Instance::instance()->unloading_time(req_2)
                                           + Instance::instance()->between_docks_time(unload_vhc, vhc_1);
                        after_req_2 = true;
                    } else if (after_req_2) {
                        if (cmp(req_start_reload + Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_), vhc_1_reload_start) <= 0) {
                            req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_] -= Instance::instance()->unloading_time(req_2);
                            req_start_unload[req_2] += Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[unload_vhc].unloaded_reqs_[i].ind_].second);
                            impacted_reloads.insert(vhc_1);
                            req_start_reload += Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_);
                            unl_pos_r2 = i;
                        } else {
                            break;
                        }
                    }
                }

            } else {
                bool before_req_2 = false;
                for (int i = schedules[unload_vhc].unloaded_reqs_.size() - 1; i >= 0; --i) {
                    if (schedules[unload_vhc].unloaded_reqs_[i].ind_ == req_2) {
                        unl_pos_r2 = i;
                        before_req_2 = true;
                    } else if (before_req_2) {
                        if (cmp(Instance::instance()->unloading_time(req_2),
                                start_reload[schedules[unload_vhc].unloaded_reqs_[i].ind_]
                                - (req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_]
                                   + Instance::instance()->unloading_time(schedules[unload_vhc].unloaded_reqs_[i].ind_)
                                   + Instance::instance()->between_docks_time(unload_vhc, solution.schedule_manager_.pic_del_[schedules[unload_vhc].unloaded_reqs_[i].ind_].second))) <= 0) {
                            req_start_unload[req_2] = req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_];
                            req_start_unload[schedules[unload_vhc].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(req_2);
                            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[unload_vhc].unloaded_reqs_[i].ind_].second);
                            impacted_reloads.insert(vhc_1);
                            unl_pos_r2 = i;
                        } else {
                            break;
                        }
                    }
                }
            }
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (rs[i].ind_ != req_1 && rs[i].ind_ != req_2) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                } else {
                    rs.erase(rs.begin() + i);
                    --i;
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == vhc_1) {
            curr_unload_end = vhc_1_unload_end;
            if (solution.schedule_manager_.pic_del_[req_2].first != vhc_1) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = req_2;
                rs[rs.size() - 1].availability_time_ = req_start_unload[req_2] + Instance::instance()->unloading_time(req_2) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[req_2].first, *rld_vhc);
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            if (solution.schedule_manager_.pic_del_[req_1].first != vhc_2) { // start reloading
                rs.resize(rs.size() + 1);
                rs[rs.size() - 1].ind_ = req_1;
                rs[rs.size() - 1].availability_time_ = req_start_unload[req_1] + Instance::instance()->unloading_time(req_1) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[req_1].first, *rld_vhc);
            }
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
            ct_delta[*rld_vhc] = curr_delta;
        } else {
            curr_delta = curr_unload_end - schedules[*rld_vhc].completion_time_;
            ct_delta[*rld_vhc] = curr_delta;
        }
    }

    for (int k = 0; k < num_vhcs; ++k) {
        delta_cost += ct_delta[k];
    }

    return delta_cost;
}

void LocalSearches::ER_move_exchange_in_best_pos_pickup_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, const int final_pos_for_n1, const int final_pos_for_n2, const int unl_pos_r1, const int unl_pos_r2) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    int node_1 = solution.routes_pickup_[vhc_1].visited_nodes_[pos_1];
    int node_2 = solution.routes_pickup_[vhc_2].visited_nodes_[pos_2];

    // delta cost on routes
    solution.routes_pickup_[vhc_1].RemoveVisitedNodeFromPos(pos_1);
    solution.routes_pickup_[vhc_1].InsertVisitedNode(final_pos_for_n2, node_2);

    solution.routes_pickup_[vhc_2].RemoveVisitedNodeFromPos(pos_2);
    solution.routes_pickup_[vhc_2].InsertVisitedNode(final_pos_for_n1, node_1);

    Schedules & schedules = solution.schedule_manager_.schedules_;

    // unloading updates
    if (solution.schedule_manager_.pic_del_[node_1].first != solution.schedule_manager_.pic_del_[node_1].second) {
        for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
            if (schedules[vhc_1].unloaded_reqs_[i].ind_ == node_1) {
                schedules[vhc_1].unloaded_reqs_.erase(schedules[vhc_1].unloaded_reqs_.begin() + i);
                break;
            }
        }
    }
    if (solution.schedule_manager_.pic_del_[node_2].first != solution.schedule_manager_.pic_del_[node_2].second) {
        for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
            if (schedules[vhc_2].unloaded_reqs_[i].ind_ == node_2) {
                schedules[vhc_2].unloaded_reqs_.erase(schedules[vhc_2].unloaded_reqs_.begin() + i);
                break;
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[node_1].first != solution.schedule_manager_.pic_del_[node_1].second) {
        solution.schedule_manager_.pic_del_[node_1].first = vhc_2;
        if (solution.schedule_manager_.pic_del_[node_1].first != solution.schedule_manager_.pic_del_[node_1].second) {
            schedules[vhc_2].unloaded_reqs_.insert(schedules[vhc_2].unloaded_reqs_.begin() + unl_pos_r1, UnloadUnit(node_1));
        } else {
            for (uint i = 0; i < schedules[vhc_2].reloaded_reqs_.size(); ++i) {
                if (schedules[vhc_2].reloaded_reqs_[i].ind_ == node_1) {
                    schedules[vhc_2].reloaded_reqs_.erase(schedules[vhc_2].reloaded_reqs_.begin() + i);
                    break;
                }
            }
        }
    } else {
        solution.schedule_manager_.pic_del_[node_1].first = vhc_2;
        schedules[vhc_2].unloaded_reqs_.insert(schedules[vhc_2].unloaded_reqs_.begin() + unl_pos_r1, UnloadUnit(node_1));
        schedules[solution.schedule_manager_.pic_del_[node_1].second].reloaded_reqs_.push_back(ReloadUnit(node_1));
    }

    if (solution.schedule_manager_.pic_del_[node_2].first != solution.schedule_manager_.pic_del_[node_2].second) {
        solution.schedule_manager_.pic_del_[node_2].first = vhc_1;
        if (solution.schedule_manager_.pic_del_[node_2].first != solution.schedule_manager_.pic_del_[node_2].second) {
            schedules[vhc_1].unloaded_reqs_.insert(schedules[vhc_1].unloaded_reqs_.begin() + unl_pos_r2, UnloadUnit(node_2));
        } else {
            for (uint i = 0; i < schedules[vhc_1].reloaded_reqs_.size(); ++i) {
                if (schedules[vhc_1].reloaded_reqs_[i].ind_ == node_2) {
                    schedules[vhc_1].reloaded_reqs_.erase(schedules[vhc_1].reloaded_reqs_.begin() + i);
                    break;
                }
            }
        }
    } else {
        solution.schedule_manager_.pic_del_[node_2].first = vhc_1;
        schedules[vhc_1].unloaded_reqs_.insert(schedules[vhc_1].unloaded_reqs_.begin() + unl_pos_r2, UnloadUnit(node_2));
        schedules[solution.schedule_manager_.pic_del_[node_2].second].reloaded_reqs_.push_back(ReloadUnit(node_2));
    }

    schedules[vhc_1].start_time_ = solution.routes_pickup_[vhc_1].cost_;
    schedules[vhc_2].start_time_ = solution.routes_pickup_[vhc_2].cost_;

    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);

    aux_start_unload = schedules[vhc_1].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
        schedules[vhc_1].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[vhc_1].unloaded_reqs_[i].ind_) {

                schedules[solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[vhc_1].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[vhc_1].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(vhc_1, solution.schedule_manager_.pic_del_[schedules[vhc_1].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[vhc_1].unloaded_reqs_.empty()) {
        schedules[vhc_1].unload_end_ = schedules[vhc_1].start_time_;
    } else {
        schedules[vhc_1].unload_end_ = aux_start_unload;
    }

    aux_start_unload = schedules[vhc_2].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
        schedules[vhc_2].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[vhc_2].unloaded_reqs_[i].ind_) {

                schedules[solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[vhc_2].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[vhc_2].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(vhc_2, solution.schedule_manager_.pic_del_[schedules[vhc_2].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[vhc_2].unloaded_reqs_.empty()) {
        schedules[vhc_2].unload_end_ = schedules[vhc_2].start_time_;
    } else {
        schedules[vhc_2].unload_end_ = aux_start_unload;
    }

    double infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

            schedules[*it].completion_time_ = schedules[*it].reloaded_reqs_.back().start_time_ + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_.back().ind_);
        } else {
            schedules[*it].completion_time_ = schedules[*it].unload_end_;
        }
    }

    solution.cost_ = 0.0;
    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution.cost_ += solution.routes_delivery_[k].cost_ + schedules[k].completion_time_;
    }
}

void LocalSearches::ER_move_exchange_in_best_pos_delivery_routes(Solution & solution, const int vhc_1, const int pos_1, const int vhc_2, const int pos_2, const int final_pos_for_n1, const int final_pos_for_n2, const int unl_pos_r1, const int unl_pos_r2) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    int node_1 = solution.routes_delivery_[vhc_1].visited_nodes_[pos_1];
    int node_2 = solution.routes_delivery_[vhc_2].visited_nodes_[pos_2];
    int req_1 = node_1 - num_reqs;
    int req_2 = node_2 - num_reqs;

    // delta cost on routes
    solution.routes_delivery_[vhc_1].RemoveVisitedNodeFromPos(pos_1);
    solution.routes_delivery_[vhc_1].InsertVisitedNode(final_pos_for_n2, node_2);

    solution.routes_delivery_[vhc_2].RemoveVisitedNodeFromPos(pos_2);
    solution.routes_delivery_[vhc_2].InsertVisitedNode(final_pos_for_n1, node_1);

    Schedules & schedules = solution.schedule_manager_.schedules_;

    // unloading-reloading updates
    solution.schedule_manager_.pic_del_[req_1].second = vhc_2;
    solution.schedule_manager_.pic_del_[req_2].second = vhc_1;

    if (solution.schedule_manager_.pic_del_[req_1].first != vhc_1) {
        for (uint i = 0; i < schedules[vhc_1].reloaded_reqs_.size(); ++i) {
            if (schedules[vhc_1].reloaded_reqs_[i].ind_ == req_1) {
                schedules[vhc_1].reloaded_reqs_.erase(schedules[vhc_1].reloaded_reqs_.begin() + i);
                break;
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[req_2].first != vhc_2) {
        for (uint i = 0; i < schedules[vhc_2].reloaded_reqs_.size(); ++i) {
            if (schedules[vhc_2].reloaded_reqs_[i].ind_ == req_2) {
                schedules[vhc_2].reloaded_reqs_.erase(schedules[vhc_2].reloaded_reqs_.begin() + i);
                break;
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[req_1].first == vhc_2) {
        for (uint i = 0; i < schedules[vhc_2].unloaded_reqs_.size(); ++i) {
            if (schedules[vhc_2].unloaded_reqs_[i].ind_ == req_1) {
                schedules[vhc_2].unloaded_reqs_.erase(schedules[vhc_2].unloaded_reqs_.begin() + i);
                break;
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[req_2].first == vhc_1) {
        for (uint i = 0; i < schedules[vhc_1].unloaded_reqs_.size(); ++i) {
            if (schedules[vhc_1].unloaded_reqs_[i].ind_ == req_2) {
                schedules[vhc_1].unloaded_reqs_.erase(schedules[vhc_1].unloaded_reqs_.begin() + i);
                break;
            }
        }
    }

    if (solution.schedule_manager_.pic_del_[req_1].first == vhc_1) {
        schedules[vhc_2].reloaded_reqs_.push_back(ReloadUnit(req_1));
        schedules[vhc_1].unloaded_reqs_.insert(schedules[vhc_1].unloaded_reqs_.begin() + unl_pos_r1, UnloadUnit(req_1));
    }

    if (solution.schedule_manager_.pic_del_[req_2].first == vhc_2) {
        schedules[vhc_1].reloaded_reqs_.push_back(ReloadUnit(req_2));
        schedules[vhc_2].unloaded_reqs_.insert(schedules[vhc_2].unloaded_reqs_.begin() + unl_pos_r2, UnloadUnit(req_2));
    }

    if (solution.schedule_manager_.pic_del_[req_1].first != vhc_1 && solution.schedule_manager_.pic_del_[req_1].first != vhc_2
        && solution.schedule_manager_.pic_del_[req_1].first == solution.schedule_manager_.pic_del_[req_2].first) {

        schedules[vhc_1].reloaded_reqs_.push_back(ReloadUnit(req_2));
        schedules[vhc_2].reloaded_reqs_.push_back(ReloadUnit(req_1));

        for (int i = schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.size() - 1; i >=0; --i) {
            if (schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_[i].ind_ == req_1
                || schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_[i].ind_ == req_2) {
                
                schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.erase(schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.begin() + i);
            }
        }

        if (unl_pos_r1 < unl_pos_r2) {
            schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.begin() + unl_pos_r1, UnloadUnit(req_1));
            schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.begin() + unl_pos_r2, UnloadUnit(req_2));
        } else {
            schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.begin() + unl_pos_r2, UnloadUnit(req_2));
            schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.begin() + unl_pos_r1, UnloadUnit(req_1));
        }
    }

    if (solution.schedule_manager_.pic_del_[req_1].first != vhc_1 && solution.schedule_manager_.pic_del_[req_1].first != vhc_2
        && solution.schedule_manager_.pic_del_[req_1].first != solution.schedule_manager_.pic_del_[req_2].first) {

        schedules[vhc_2].reloaded_reqs_.push_back(ReloadUnit(req_1));
        for (int i = schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.size() - 1; i >=0; --i) {
            if (schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_[i].ind_ == req_1) {
                schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.erase(schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.begin() + i);
                break;
            }
        }
        schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[req_1].first].unloaded_reqs_.begin() + unl_pos_r1, UnloadUnit(req_1));
    }

    if (solution.schedule_manager_.pic_del_[req_2].first != vhc_1 && solution.schedule_manager_.pic_del_[req_2].first != vhc_2
        && solution.schedule_manager_.pic_del_[req_1].first != solution.schedule_manager_.pic_del_[req_2].first) {

        schedules[vhc_1].reloaded_reqs_.push_back(ReloadUnit(req_2));
        for (int i = schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_.size() - 1; i >=0; --i) {
            if (schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_[i].ind_ == req_2) {
                schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_.erase(schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_.begin() + i);
                break;
            }
        }
        schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_.insert(schedules[solution.schedule_manager_.pic_del_[req_2].first].unloaded_reqs_.begin() + unl_pos_r2, UnloadUnit(req_2));
    }

    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(vhc_1);
    impacted_reloads.insert(vhc_2);

    set<int> pic_vhcs;
    pic_vhcs.insert(solution.schedule_manager_.pic_del_[req_1].first);
    pic_vhcs.insert(solution.schedule_manager_.pic_del_[req_2].first);

    for (auto v = pic_vhcs.begin(); v != pic_vhcs.end(); ++v) {
        aux_start_unload = schedules[*v].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[*v].unloaded_reqs_.size(); ++i) {
            schedules[*v].unloaded_reqs_[i].start_time_ = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[*v].unloaded_reqs_[i].ind_);
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[*v].unloaded_reqs_[i].ind_].second);

            for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[*v].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
                if (schedules[solution.schedule_manager_.pic_del_[schedules[*v].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                    == schedules[*v].unloaded_reqs_[i].ind_) {

                    schedules[solution.schedule_manager_.pic_del_[schedules[*v].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                            schedules[*v].unloaded_reqs_[i].start_time_
                            + Instance::instance()->unloading_time(schedules[*v].unloaded_reqs_[i].ind_)
                            + Instance::instance()->between_docks_time(*v, solution.schedule_manager_.pic_del_[schedules[*v].unloaded_reqs_[i].ind_].second);
                }
            }
        }
        if (schedules[*v].unloaded_reqs_.empty()) {
            schedules[*v].unload_end_ = schedules[*v].start_time_;
        } else {
            schedules[*v].unload_end_ = aux_start_unload;
        }
    }

    double infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

            schedules[*it].completion_time_ = schedules[*it].reloaded_reqs_.back().start_time_ + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_.back().ind_);
        } else {
            schedules[*it].completion_time_ = schedules[*it].unload_end_;
        }
    }

    solution.cost_ = 0.0;
    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution.cost_ += solution.routes_delivery_[k].cost_ + schedules[k].completion_time_;
    }
}
