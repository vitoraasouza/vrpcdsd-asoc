#include "LocalSearches.h"

bool LocalSearches::ER_Parallel_exchange_in_best_pos(Solution & solution, LS_strat search_strat) {
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    bool improve, general_improvement;
    int k1_pic, k1_del, k2_pic, k2_del;
    int pos_r1_pic, pos_r1_del, pos_r2_pic, pos_r2_del;
    int best_pos_r1_pic = -1, best_pos_r1_del = -1, best_pos_r2_pic = -1, best_pos_r2_del = -1;
    int best_k1_pic = -1, best_k1_del = -1, best_k2_pic = -1, best_k2_del = -1;
    int free_space_k1_pic, free_space_k1_del, free_space_k2_pic, free_space_k2_del, r1_demand, r2_demand;
    double best_cost, temp_cost, current_cost;
    int fpos_r1_pic, fpos_r2_pic, fpos_r1_del, fpos_r2_del, unl_pos_r1, unl_pos_r2;
    int best_fpos_r1_pic = -1, best_fpos_r2_pic = -1, best_fpos_r1_del = -1, best_fpos_r2_del = -1, best_unl_pos_r1 = -1, best_unl_pos_r2 = -1;
    vector<bool> reqs_already_selected(num_requests + 1);
    Routes & pic_routes = solution.routes_pickup_;
    Routes & del_routes = solution.routes_delivery_;

    vector<int> selectable_reqs(num_requests);
    iota(selectable_reqs.begin(), selectable_reqs.end(), 1);
    shuffle(selectable_reqs.begin(), selectable_reqs.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;

    while (improve) {
        improve = false;
        best_cost = solution.cost_;
        reqs_already_selected.assign(num_requests + 1, false);
        for (auto req_1 = selectable_reqs.begin(); req_1 != selectable_reqs.end(); ++req_1) {

            reqs_already_selected[*req_1] = true;
            k1_pic = solution.schedule_manager_.pic_del_[*req_1].first;
            k1_del = solution.schedule_manager_.pic_del_[*req_1].second;
            pos_r1_pic = find(pic_routes[k1_pic].visited_nodes_.begin(), pic_routes[k1_pic].visited_nodes_.end(), *req_1) - pic_routes[k1_pic].visited_nodes_.begin();
            pos_r1_del = find(del_routes[k1_del].visited_nodes_.begin(), del_routes[k1_del].visited_nodes_.end(), *req_1 + num_requests) - del_routes[k1_del].visited_nodes_.begin();

            r1_demand = Instance::instance()->node(*req_1).demand();
            free_space_k1_pic = vhc_capacity - pic_routes[k1_pic].load_ + r1_demand;
            free_space_k1_del = vhc_capacity - del_routes[k1_del].load_ + r1_demand;

            temp_cost = solution.cost_;

            for (int req_2 = 1; req_2 <= num_requests; ++req_2) {

                if (!reqs_already_selected[req_2]) {
                    k2_pic = solution.schedule_manager_.pic_del_[req_2].first;
                    k2_del = solution.schedule_manager_.pic_del_[req_2].second;

                    if (k1_pic != k2_pic && k1_del != k2_del) {
                        r2_demand = Instance::instance()->node(req_2).demand();
                        free_space_k2_pic = vhc_capacity - pic_routes[k2_pic].load_ + r2_demand;
                        free_space_k2_del = vhc_capacity - del_routes[k2_del].load_ + r2_demand;

                        if (free_space_k1_pic >= r2_demand && free_space_k2_pic >= r1_demand && free_space_k1_del >= r2_demand && free_space_k2_del >= r1_demand) {

                            pos_r2_pic = find(pic_routes[k2_pic].visited_nodes_.begin(), pic_routes[k2_pic].visited_nodes_.end(), req_2) - pic_routes[k2_pic].visited_nodes_.begin();
                            pos_r2_del = find(del_routes[k2_del].visited_nodes_.begin(), del_routes[k2_del].visited_nodes_.end(), req_2 + num_requests) - del_routes[k2_del].visited_nodes_.begin();

                            current_cost = temp_cost + ER_parallel_exchange_in_best_pos_delta_cost(solution, k1_pic, pos_r1_pic, k1_del, pos_r1_del, k2_pic, pos_r2_pic, k2_del, pos_r2_del,
                                                                                                   fpos_r1_pic, fpos_r2_pic, fpos_r1_del, fpos_r2_del, unl_pos_r1, unl_pos_r2);

                            if (cmp(current_cost, best_cost) < 0) {
                                best_cost = current_cost;
                                best_pos_r1_pic = pos_r1_pic;
                                best_pos_r1_del = pos_r1_del;
                                best_pos_r2_pic = pos_r2_pic;
                                best_pos_r2_del = pos_r2_del;
                                best_k1_pic = k1_pic;
                                best_k1_del = k1_del;
                                best_k2_pic = k2_pic;
                                best_k2_del = k2_del;
                                best_fpos_r1_pic = fpos_r1_pic;
                                best_fpos_r1_del = fpos_r1_del;
                                best_fpos_r2_pic = fpos_r2_pic;
                                best_fpos_r2_del = fpos_r2_del;
                                best_unl_pos_r1 = unl_pos_r1;
                                best_unl_pos_r2 = unl_pos_r2;
                                improve = true;
                            }
                        }
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;

                ER_move_parallel_exchange_in_best_pos(solution, best_k1_pic, best_pos_r1_pic, best_k1_del, best_pos_r1_del, best_k2_pic, best_pos_r2_pic, best_k2_del, best_pos_r2_del,
                                                      best_fpos_r1_pic, best_fpos_r2_pic, best_fpos_r1_del, best_fpos_r2_del, best_unl_pos_r1, best_unl_pos_r2);

                if (search_strat == LS_strat::complete_first_improv_) {
                    shuffle(selectable_reqs.begin(), selectable_reqs.end(), rand_utils::generator);
                    break;
                } else {
                    return general_improvement;
                }
            }

        }

        if (improve && (search_strat == LS_strat::complete_best_improv_ || search_strat == LS_strat::one_best_improv_)) {
            general_improvement = true;

            ER_move_parallel_exchange_in_best_pos(solution, best_k1_pic, best_pos_r1_pic, best_k1_del, best_pos_r1_del, best_k2_pic, best_pos_r2_pic, best_k2_del, best_pos_r2_del,
                                                  best_fpos_r1_pic, best_fpos_r2_pic, best_fpos_r1_del, best_fpos_r2_del, best_unl_pos_r1, best_unl_pos_r2);

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }
    }

    return general_improvement;
}

double LocalSearches::ER_parallel_exchange_in_best_pos_delta_cost(Solution & solution, const int k1_pic, const int pos_r1_pic, const int k1_del, const int pos_r1_del,
                                                                  const int k2_pic, const int pos_r2_pic, const int k2_del, const int pos_r2_del,
                                                                  int & fpos_r1_pic, int & fpos_r2_pic, int & fpos_r1_del, int & fpos_r2_del,
                                                                  int & unl_pos_r1, int & unl_pos_r2) {

    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    int req_1 = solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic];
    int req_2 = solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic];
    double k1_pic_route_cost, k2_pic_route_cost, k1_del_route_cost, k2_del_route_cost;
    double k1_del_diff_route_cost, k2_del_diff_route_cost;
    double k1_pic_unload_end, k2_pic_unload_end;
    double best_cost, temp_cost, curr_cost;

    unl_pos_r1 = unl_pos_r2 = -1;

    // delta cost on routes
    // k1 pickup
    best_cost = 1000.0 * solution.routes_pickup_[k1_pic].cost_;
    temp_cost = solution.routes_pickup_[k1_pic].cost_
                - Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic - 1], solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic])
                - Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic], solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic + 1])
                + Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic - 1], solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic + 1]);

    for (int i = 1; i <= solution.routes_pickup_[k1_pic].last_insertable_pos_; ++i) {
        if (i == pos_r1_pic) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[i - 1], solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic])
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic], solution.routes_pickup_[k1_pic].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[i - 1], solution.routes_pickup_[k1_pic].visited_nodes_[i + 1]);
        } else if (i != pos_r1_pic + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[i - 1], solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic])
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic], solution.routes_pickup_[k1_pic].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[i - 1], solution.routes_pickup_[k1_pic].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_r1_pic) {
                fpos_r2_pic = i-1;
            } else {
                fpos_r2_pic = i;
            }
        }
    }
    k1_pic_route_cost = best_cost;

    // k2 pickup
    best_cost = 1000.0 * solution.routes_pickup_[k2_pic].cost_;
    temp_cost = solution.routes_pickup_[k2_pic].cost_
                - Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic - 1], solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic])
                - Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic], solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic + 1])
                + Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic - 1], solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic + 1]);

    for (int i = 1; i <= solution.routes_pickup_[k2_pic].last_insertable_pos_; ++i) {
        if (i == pos_r2_pic) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[i - 1], solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic])
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic], solution.routes_pickup_[k2_pic].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[i - 1], solution.routes_pickup_[k2_pic].visited_nodes_[i + 1]);
        } else if (i != pos_r2_pic + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[i - 1], solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic])
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic], solution.routes_pickup_[k2_pic].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[i - 1], solution.routes_pickup_[k2_pic].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_r2_pic) {
                fpos_r1_pic = i-1;
            } else {
                fpos_r1_pic = i;
            }
        }
    }
    k2_pic_route_cost = best_cost;

    // k1 delivery
    best_cost = 1000.0 * solution.routes_delivery_[k1_del].cost_;
    temp_cost = solution.routes_delivery_[k1_del].cost_
                - Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del - 1], solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del])
                - Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del], solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del + 1])
                + Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del - 1], solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del + 1]);

    for (int i = 1; i <= solution.routes_delivery_[k1_del].last_insertable_pos_; ++i) {
        if (i == pos_r1_del) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[i - 1], solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del])
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del], solution.routes_delivery_[k1_del].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[i - 1], solution.routes_delivery_[k1_del].visited_nodes_[i + 1]);
        } else if (i != pos_r1_del + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[i - 1], solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del])
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del], solution.routes_delivery_[k1_del].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[i - 1], solution.routes_delivery_[k1_del].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_r1_del) {
                fpos_r2_del = i-1;
            } else {
                fpos_r2_del = i;
            }
        }
    }
    k1_del_route_cost = best_cost;

    // k2 delivery
    best_cost = 1000.0 * solution.routes_delivery_[k2_del].cost_;
    temp_cost = solution.routes_delivery_[k2_del].cost_
                - Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del - 1], solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del])
                - Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del], solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del + 1])
                + Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del - 1], solution.routes_delivery_[k2_del].visited_nodes_[pos_r2_del + 1]);

    for (int i = 1; i <= solution.routes_delivery_[k2_del].last_insertable_pos_; ++i) {
        if (i == pos_r2_del) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[i - 1], solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del])
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del], solution.routes_delivery_[k2_del].visited_nodes_[i + 1])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[i - 1], solution.routes_delivery_[k2_del].visited_nodes_[i + 1]);
        } else if (i != pos_r2_del + 1) {
            curr_cost = temp_cost
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[i - 1], solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del])
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_r1_del], solution.routes_delivery_[k2_del].visited_nodes_[i])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[i - 1], solution.routes_delivery_[k2_del].visited_nodes_[i]);
        } else {
            continue;
        }

        if (cmp(curr_cost, best_cost) < 0) {
            best_cost = curr_cost;
            if (i > pos_r2_del) {
                fpos_r1_del = i-1;
            } else {
                fpos_r1_del = i;
            }
        }
    }
    k2_del_route_cost = best_cost;

    k1_del_diff_route_cost = k1_del_route_cost - solution.routes_delivery_[k1_del].cost_;
    k2_del_diff_route_cost = k2_del_route_cost - solution.routes_delivery_[k2_del].cost_;

    delta_cost = k1_del_diff_route_cost + k2_del_diff_route_cost;

    // unloading updates
    double aux_start_unload;
    Schedules & schedules = solution.schedule_manager_.schedules_;

    // on k1 pickup
    aux_start_unload = k1_pic_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k1_pic].unloaded_reqs_.size(); ++i) {
        if (schedules[k1_pic].unloaded_reqs_[i].ind_ != req_1) {
            req_start_unload[schedules[k1_pic].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_);
        }
    }

    if (k1_pic != k1_del) {
        req_start_unload[req_2] = aux_start_unload;
        k1_pic_unload_end = aux_start_unload + Instance::instance()->unloading_time(req_2);
    } else {
        req_start_unload[req_2] = 0.0;
        if (schedules[k1_pic].unloaded_reqs_.empty()
            || (schedules[k1_pic].unloaded_reqs_.size() == 1
                && schedules[k1_pic].unloaded_reqs_[0].ind_ ==  req_1)) {
            k1_pic_unload_end = k1_pic_route_cost;
        } else {
            k1_pic_unload_end = aux_start_unload;
        }
    }

    // on k2 pickup
    aux_start_unload = k2_pic_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k2_pic].unloaded_reqs_.size(); ++i) {
        if (schedules[k2_pic].unloaded_reqs_[i].ind_ != req_2) {
            req_start_unload[schedules[k2_pic].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_);
        }
    }

    if (k2_pic != k2_del) {
        req_start_unload[req_1] = aux_start_unload;
        k2_pic_unload_end = aux_start_unload + Instance::instance()->unloading_time(req_1);
    } else {
        req_start_unload[req_1] = 0.0;
        if (schedules[k2_pic].unloaded_reqs_.empty()
            || (schedules[k2_pic].unloaded_reqs_.size() == 1
                && schedules[k2_pic].unloaded_reqs_[0].ind_ ==  req_2)) {
            k2_pic_unload_end = k2_pic_route_cost;
        } else {
            k2_pic_unload_end = aux_start_unload;
        }
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(k1_pic);
    impacted_reloads.insert(k2_pic);
    impacted_reloads.insert(k1_del);
    impacted_reloads.insert(k2_del);
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
                if (rs[i].ind_ != req_1 && rs[i].ind_ != req_2) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
                } else {
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == k1_pic) {
            curr_unload_end = k1_pic_unload_end;
        } else if (*rld_vhc == k2_pic) {
            curr_unload_end = k2_pic_unload_end;
        }
        if (*rld_vhc == k1_del && k1_pic != k1_del) {
            rs.resize(rs.size() + 1);
            rs[rs.size() - 1].ind_ = req_2;
            rs[rs.size() - 1].availability_time_ = req_start_unload[req_2] + Instance::instance()->unloading_time(req_2) + Instance::instance()->between_docks_time(*rld_vhc, k1_pic);
        } else if (*rld_vhc == k2_del && k2_pic != k2_del) {
            rs.resize(rs.size() + 1);
            rs[rs.size() - 1].ind_ = req_1;
            rs[rs.size() - 1].availability_time_ = req_start_unload[req_1] + Instance::instance()->unloading_time(req_1) + Instance::instance()->between_docks_time(*rld_vhc, k2_pic);
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

    if (k2_pic != k2_del) {
        bool after_req_2 = true;
        unl_pos_r1 = schedules[k2_pic].unloaded_reqs_.size() - (after_req_2 ? 1 : 0);
        for (int i = schedules[k2_pic].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (schedules[k2_pic].unloaded_reqs_[i].ind_ != req_2
                && cmp(Instance::instance()->unloading_time(req_1),
                       start_reload[schedules[k2_pic].unloaded_reqs_[i].ind_]
                       - (req_start_unload[schedules[k2_pic].unloaded_reqs_[i].ind_]
                          + Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_)
                          + Instance::instance()->between_docks_time(k2_pic, solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second))) <= 0) {
                req_start_unload[req_1] -= Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[k2_pic].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(req_1);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(k2_del);
                unl_pos_r1 = i - (after_req_2 ? 1 : 0);
            } else if (schedules[k2_pic].unloaded_reqs_[i].ind_ == req_2) {
                after_req_2 = false;
            } else {
                break;
            }
        }
    }

    if (k1_pic != k1_del) {
        bool after_req_1 = true;
        unl_pos_r2 = schedules[k1_pic].unloaded_reqs_.size() - (after_req_1 ? 1 : 0);
        for (int i = schedules[k1_pic].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (schedules[k1_pic].unloaded_reqs_[i].ind_ != req_1
                && cmp(Instance::instance()->unloading_time(req_2),
                       start_reload[schedules[k1_pic].unloaded_reqs_[i].ind_]
                       - (req_start_unload[schedules[k1_pic].unloaded_reqs_[i].ind_]
                          + Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_)
                          + Instance::instance()->between_docks_time(k1_pic, solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second))) <= 0) {
                req_start_unload[req_2] -= Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[k1_pic].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(req_2);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(k1_del);
                unl_pos_r2 = i - (after_req_1 ? 1 : 0);
            } else if (schedules[k1_pic].unloaded_reqs_[i].ind_ == req_1) {
                after_req_1 = false;
            } else {
                break;
            }
        }
    }

    for (auto rld_vhc = impacted_reloads.begin(); rld_vhc != impacted_reloads.end(); ++rld_vhc) {
        ReloadSchedule rs = schedules[*rld_vhc].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                if (rs[i].ind_ != req_1 && rs[i].ind_ != req_2) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
                } else {
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == k1_pic) {
            curr_unload_end = k1_pic_unload_end;
        } else if (*rld_vhc == k2_pic) {
            curr_unload_end = k2_pic_unload_end;
        }
        if (*rld_vhc == k1_del && k1_pic != k1_del) {
            rs.resize(rs.size() + 1);
            rs[rs.size() - 1].ind_ = req_2;
            rs[rs.size() - 1].availability_time_ = req_start_unload[req_2] + Instance::instance()->unloading_time(req_2) + Instance::instance()->between_docks_time(*rld_vhc, k1_pic);
        } else if (*rld_vhc == k2_del && k2_pic != k2_del) {
            rs.resize(rs.size() + 1);
            rs[rs.size() - 1].ind_ = req_1;
            rs[rs.size() - 1].availability_time_ = req_start_unload[req_1] + Instance::instance()->unloading_time(req_1) + Instance::instance()->between_docks_time(*rld_vhc, k2_pic);
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

void LocalSearches::ER_move_parallel_exchange_in_best_pos(Solution & solution, const int k1_pic, const int pos_r1_pic, const int k1_del, const int pos_r1_del,
                                                          const int k2_pic, const int pos_r2_pic, const int k2_del, const int pos_r2_del,
                                                          const int fpos_r1_pic, const int fpos_r2_pic, const int fpos_r1_del, const int fpos_r2_del,
                                                          const int unl_pos_r1, const int unl_pos_r2) {

    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    int req_1 = solution.routes_pickup_[k1_pic].visited_nodes_[pos_r1_pic];
    int req_2 = solution.routes_pickup_[k2_pic].visited_nodes_[pos_r2_pic];

    // delta cost on routes
    solution.routes_pickup_[k1_pic].RemoveVisitedNodeFromPos(pos_r1_pic);
    solution.routes_pickup_[k1_pic].InsertVisitedNode(fpos_r2_pic, req_2);

    solution.routes_pickup_[k2_pic].RemoveVisitedNodeFromPos(pos_r2_pic);
    solution.routes_pickup_[k2_pic].InsertVisitedNode(fpos_r1_pic, req_1);

    solution.routes_delivery_[k1_del].RemoveVisitedNodeFromPos(pos_r1_del);
    solution.routes_delivery_[k1_del].InsertVisitedNode(fpos_r2_del, req_2 + num_reqs);

    solution.routes_delivery_[k2_del].RemoveVisitedNodeFromPos(pos_r2_del);
    solution.routes_delivery_[k2_del].InsertVisitedNode(fpos_r1_del, req_1 + num_reqs);

    Schedules & schedules = solution.schedule_manager_.schedules_;

    // unloading updates
    if (k1_pic != k1_del) {
        for (uint i = 0; i < schedules[k1_pic].unloaded_reqs_.size(); ++i) {
            if (schedules[k1_pic].unloaded_reqs_[i].ind_ == req_1) {
                schedules[k1_pic].unloaded_reqs_.erase(schedules[k1_pic].unloaded_reqs_.begin() + i);
                break;
            }
        }

        for (uint i = 0; i < schedules[k1_del].reloaded_reqs_.size(); ++i) {
            if (schedules[k1_del].reloaded_reqs_[i].ind_ == req_1) {
                schedules[k1_del].reloaded_reqs_.erase(schedules[k1_del].reloaded_reqs_.begin() + i);
                break;
            }
        }

        schedules[k1_pic].unloaded_reqs_.insert(schedules[k1_pic].unloaded_reqs_.begin() + unl_pos_r2, UnloadUnit(req_2));
        schedules[k1_del].reloaded_reqs_.push_back(ReloadUnit(req_2));
    }
    solution.schedule_manager_.pic_del_[req_2].first = k1_pic;
    solution.schedule_manager_.pic_del_[req_2].second = k1_del;

    if (k2_pic != k2_del) {
        for (uint i = 0; i < schedules[k2_pic].unloaded_reqs_.size(); ++i) {
            if (schedules[k2_pic].unloaded_reqs_[i].ind_ == req_2) {
                schedules[k2_pic].unloaded_reqs_.erase(schedules[k2_pic].unloaded_reqs_.begin() + i);
                break;
            }
        }

        for (uint i = 0; i < schedules[k2_del].reloaded_reqs_.size(); ++i) {
            if (schedules[k2_del].reloaded_reqs_[i].ind_ == req_2) {
                schedules[k2_del].reloaded_reqs_.erase(schedules[k2_del].reloaded_reqs_.begin() + i);
                break;
            }
        }

        schedules[k2_pic].unloaded_reqs_.insert(schedules[k2_pic].unloaded_reqs_.begin() + unl_pos_r1, UnloadUnit(req_1));
        schedules[k2_del].reloaded_reqs_.push_back(ReloadUnit(req_1));
    }
    solution.schedule_manager_.pic_del_[req_1].first = k2_pic;
    solution.schedule_manager_.pic_del_[req_1].second = k2_del;

    schedules[k1_pic].start_time_ = solution.routes_pickup_[k1_pic].cost_;
    schedules[k2_pic].start_time_ = solution.routes_pickup_[k2_pic].cost_;

    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(k1_pic);
    impacted_reloads.insert(k2_pic);

    aux_start_unload = schedules[k1_pic].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k1_pic].unloaded_reqs_.size(); ++i) {
        schedules[k1_pic].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[k1_pic].unloaded_reqs_[i].ind_) {

                schedules[solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[k1_pic].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(k1_pic, solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[k1_pic].unloaded_reqs_.empty()) {
        schedules[k1_pic].unload_end_ = schedules[k1_pic].start_time_;
    } else {
        schedules[k1_pic].unload_end_ = aux_start_unload;
    }

    aux_start_unload = schedules[k2_pic].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k2_pic].unloaded_reqs_.size(); ++i) {
        schedules[k2_pic].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[k2_pic].unloaded_reqs_[i].ind_) {

                schedules[solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[k2_pic].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(k2_pic, solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[k2_pic].unloaded_reqs_.empty()) {
        schedules[k2_pic].unload_end_ = schedules[k2_pic].start_time_;
    } else {
        schedules[k2_pic].unload_end_ = aux_start_unload;
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

bool LocalSearches::ER_Synced_reinsertion(Solution & solution, LS_strat search_strat) {
    int num_requests = Instance::instance()->num_requests();
    int num_vehicles = Instance::instance()->num_vehicles();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    bool improve, general_improvement;
    int k1_pic, k1_del;
    int pos_k1_pic, pos_k1_del, pos_k2_pic, pos_k2_del;
    int best_pos_k1_pic = -1, best_pos_k1_del = -1, best_pos_k2_pic = -1, best_pos_k2_del = -1;
    int best_k1_pic = -1, best_k1_del = -1, best_k2_pic = -1, best_k2_del = -1;
    int req_demand, unl_pos, best_unl_pos = -1;
    double best_cost, temp_cost, current_cost;
    Routes & pic_routes = solution.routes_pickup_;
    Routes & del_routes = solution.routes_delivery_;

    vector<int> selectable_reqs(num_requests);
    iota(selectable_reqs.begin(), selectable_reqs.end(), 1);
    shuffle(selectable_reqs.begin(), selectable_reqs.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;

    while (improve) {
        improve = false;
        best_cost = solution.cost_;
        for (int req : selectable_reqs) {

            k1_pic = solution.schedule_manager_.pic_del_[req].first;
            k1_del = solution.schedule_manager_.pic_del_[req].second;

            if (pic_routes[k1_pic].num_nodes_ == 1 || del_routes[k1_del].num_nodes_ == 1) {
                continue;
            }

            pos_k1_pic = find(pic_routes[k1_pic].visited_nodes_.begin(), pic_routes[k1_pic].visited_nodes_.end(), req) - pic_routes[k1_pic].visited_nodes_.begin();
            pos_k1_del = find(del_routes[k1_del].visited_nodes_.begin(), del_routes[k1_del].visited_nodes_.end(), req + num_requests) - del_routes[k1_del].visited_nodes_.begin();

            req_demand = Instance::instance()->node(req).demand();

            temp_cost = solution.cost_;

            for (int k2_pic = 0; k2_pic < num_vehicles; ++k2_pic) {
                if (k2_pic != k1_pic && pic_routes[k2_pic].load_ + req_demand <= vhc_capacity) {
                    pos_k2_pic = ER_1opt_best_insertion_pos(pic_routes[k2_pic].visited_nodes_, req);

                    for (int k2_del = 0; k2_del < num_vehicles; ++k2_del) {
                        if (k2_del != k1_del && del_routes[k2_del].load_ + req_demand <= vhc_capacity) {
                            pos_k2_del = ER_1opt_best_insertion_pos(del_routes[k2_del].visited_nodes_, req + num_requests);

                            current_cost = temp_cost + ER_Synced_reinsertion_delta_cost(solution, k1_pic, pos_k1_pic, k1_del, pos_k1_del, k2_pic, pos_k2_pic, k2_del, pos_k2_del, unl_pos);

                            if (cmp(current_cost, best_cost) < 0) {
                                best_cost = current_cost;
                                best_pos_k1_pic = pos_k1_pic;
                                best_pos_k1_del = pos_k1_del;
                                best_pos_k2_pic = pos_k2_pic;
                                best_pos_k2_del = pos_k2_del;
                                best_k1_pic = k1_pic;
                                best_k1_del = k1_del;
                                best_k2_pic = k2_pic;
                                best_k2_del = k2_del;
                                best_unl_pos = unl_pos;
                                improve = true;
                            }
                        }
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;

                ER_move_synced_reinsertion(solution, best_k1_pic, best_pos_k1_pic, best_k1_del, best_pos_k1_del, best_k2_pic, best_pos_k2_pic, best_k2_del, best_pos_k2_del, best_unl_pos);

                if (search_strat == LS_strat::complete_first_improv_) {
                    shuffle(selectable_reqs.begin(), selectable_reqs.end(), rand_utils::generator);
                    break;
                } else {
                    return general_improvement;
                }
            }

        }

        if (improve && (search_strat == LS_strat::complete_best_improv_ || search_strat == LS_strat::one_best_improv_)) {
            general_improvement = true;

            ER_move_synced_reinsertion(solution, best_k1_pic, best_pos_k1_pic, best_k1_del, best_pos_k1_del, best_k2_pic, best_pos_k2_pic, best_k2_del, best_pos_k2_del, best_unl_pos);

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }
    }

    return general_improvement;
}

double LocalSearches::ER_Synced_reinsertion_delta_cost(Solution & solution, const int k1_pic, const int pos_k1_pic, const int k1_del, const int pos_k1_del,
                                                       const int k2_pic, const int pos_k2_pic, const int k2_del, const int pos_k2_del,
                                                       int & unl_pos) {

    int num_reqs = Instance::instance()->num_requests();
    int num_vhcs = Instance::instance()->num_vehicles();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    double delta_cost = 0.0;
    int req_1 = solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic];
    double k1_pic_route_cost, k2_pic_route_cost, k1_del_route_cost, k2_del_route_cost;
    double k1_del_diff_route_cost, k2_del_diff_route_cost;
    double k1_pic_unload_end, k2_pic_unload_end;

    unl_pos = -1;

    // delta cost on routes
    // k1 pickup
    k1_pic_route_cost = solution.routes_pickup_[k1_pic].cost_
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic - 1], solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic])
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic], solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic + 1])
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic - 1], solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic + 1]);

    // k1 delivery
    k1_del_route_cost = solution.routes_delivery_[k1_del].cost_
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_k1_del - 1], solution.routes_delivery_[k1_del].visited_nodes_[pos_k1_del])
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_k1_del], solution.routes_delivery_[k1_del].visited_nodes_[pos_k1_del + 1])
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k1_del].visited_nodes_[pos_k1_del - 1], solution.routes_delivery_[k1_del].visited_nodes_[pos_k1_del + 1]);

    // k2 pickup
    k2_pic_route_cost = solution.routes_pickup_[k2_pic].cost_
                        - Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[pos_k2_pic - 1], solution.routes_pickup_[k2_pic].visited_nodes_[pos_k2_pic])
                        + Instance::instance()->EdgeCost(solution.routes_pickup_[k2_pic].visited_nodes_[pos_k2_pic - 1], req_1)
                        + Instance::instance()->EdgeCost(req_1, solution.routes_pickup_[k2_pic].visited_nodes_[pos_k2_pic]);

    // k2 delivery
    k2_del_route_cost = solution.routes_delivery_[k2_del].cost_
                        - Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[pos_k2_del - 1], solution.routes_delivery_[k2_del].visited_nodes_[pos_k2_del])
                        + Instance::instance()->EdgeCost(solution.routes_delivery_[k2_del].visited_nodes_[pos_k2_del - 1], req_1 + num_reqs)
                        + Instance::instance()->EdgeCost(req_1 + num_reqs, solution.routes_delivery_[k2_del].visited_nodes_[pos_k2_del]);

    k1_del_diff_route_cost = k1_del_route_cost - solution.routes_delivery_[k1_del].cost_;
    k2_del_diff_route_cost = k2_del_route_cost - solution.routes_delivery_[k2_del].cost_;

    delta_cost = k1_del_diff_route_cost + k2_del_diff_route_cost;

    // unloading updates
    double aux_start_unload;
    Schedules & schedules = solution.schedule_manager_.schedules_;

    // on k1 pickup
    aux_start_unload = k1_pic_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k1_pic].unloaded_reqs_.size(); ++i) {
        if (schedules[k1_pic].unloaded_reqs_[i].ind_ != req_1) {
            req_start_unload[schedules[k1_pic].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_);
        }
    }

    if (schedules[k1_pic].unloaded_reqs_.empty()
        || (schedules[k1_pic].unloaded_reqs_.size() == 1
            && schedules[k1_pic].unloaded_reqs_[0].ind_ ==  req_1)) {
        k1_pic_unload_end = k1_pic_route_cost;
    } else {
        k1_pic_unload_end = aux_start_unload;
    }

    // on k2 pickup
    aux_start_unload = k2_pic_route_cost + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k2_pic].unloaded_reqs_.size(); ++i) {
        req_start_unload[schedules[k2_pic].unloaded_reqs_[i].ind_] = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_);
    }

    if (k2_pic != k2_del) {
        req_start_unload[req_1] = aux_start_unload;
        k2_pic_unload_end = aux_start_unload + Instance::instance()->unloading_time(req_1);
    } else {
        req_start_unload[req_1] = 0.0;
        if (schedules[k2_pic].unloaded_reqs_.empty()) {
            k2_pic_unload_end = k2_pic_route_cost;
        } else {
            k2_pic_unload_end = aux_start_unload;
        }
    }

    set<int> impacted_reloads;
    impacted_reloads.insert(k1_pic);
    impacted_reloads.insert(k1_del);
    impacted_reloads.insert(k2_pic);
    impacted_reloads.insert(k2_del);
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
                if (rs[i].ind_ != req_1) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                } else {
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == k1_pic) {
            curr_unload_end = k1_pic_unload_end;
        } else if (*rld_vhc == k2_pic) {
            curr_unload_end = k2_pic_unload_end;
        }
        if (*rld_vhc == k2_del && k2_pic != k2_del) {
            rs.resize(rs.size() + 1);
            rs[rs.size() - 1].ind_ = req_1;
            rs[rs.size() - 1].availability_time_ = req_start_unload[req_1] + Instance::instance()->unloading_time(req_1) + Instance::instance()->between_docks_time(k2_pic, *rld_vhc);
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

    if (k2_pic != k2_del) {
        unl_pos = schedules[k2_pic].unloaded_reqs_.size();
        for (int i = schedules[k2_pic].unloaded_reqs_.size() - 1; i >= 0; --i) {
            if (cmp(Instance::instance()->unloading_time(req_1),
                    start_reload[schedules[k2_pic].unloaded_reqs_[i].ind_]
                    - (req_start_unload[schedules[k2_pic].unloaded_reqs_[i].ind_]
                       + Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_)
                       + Instance::instance()->between_docks_time(k2_pic, solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second))) <= 0) {
                req_start_unload[req_1] -= Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_);
                req_start_unload[schedules[k2_pic].unloaded_reqs_[i].ind_] += Instance::instance()->unloading_time(req_1);
                impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second);
                impacted_reloads.insert(k2_del);
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
                if (rs[i].ind_ != req_1) {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(solution.schedule_manager_.pic_del_[rs[i].ind_].first, *rld_vhc);
                } else {
                    // remover posicao i
                    rs.erase(rs.begin() + i);
                    --i;
                }
            }
        }

        curr_unload_end = schedules[*rld_vhc].unload_end_;
        if (*rld_vhc == k1_pic) {
            curr_unload_end = k1_pic_unload_end;
        } else if (*rld_vhc == k2_pic) {
            curr_unload_end = k2_pic_unload_end;
        }
        if (*rld_vhc == k2_del && k2_pic != k2_del) {
            rs.resize(rs.size() + 1);
            rs[rs.size() - 1].ind_ = req_1;
            rs[rs.size() - 1].availability_time_ = req_start_unload[req_1] + Instance::instance()->unloading_time(req_1) + Instance::instance()->between_docks_time(k2_pic, *rld_vhc);
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

void LocalSearches::ER_move_synced_reinsertion(Solution & solution, const int k1_pic, const int pos_k1_pic, const int k1_del, const int pos_k1_del,
                                               const int k2_pic, const int pos_k2_pic, const int k2_del, const int pos_k2_del,
                                               const int unl_pos) {

    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    int req_1 = solution.routes_pickup_[k1_pic].visited_nodes_[pos_k1_pic];

    // delta cost on routes
    solution.routes_pickup_[k1_pic].RemoveVisitedNodeFromPos(pos_k1_pic);
    solution.routes_delivery_[k1_del].RemoveVisitedNodeFromPos(pos_k1_del);

    solution.routes_pickup_[k2_pic].InsertVisitedNode(pos_k2_pic, req_1);
    solution.routes_delivery_[k2_del].InsertVisitedNode(pos_k2_del, req_1 + num_reqs);

    Schedules & schedules = solution.schedule_manager_.schedules_;

    // unloading updates
    if (k1_pic != k1_del) {
        for (uint i = 0; i < schedules[k1_pic].unloaded_reqs_.size(); ++i) {
            if (schedules[k1_pic].unloaded_reqs_[i].ind_ == req_1) {
                schedules[k1_pic].unloaded_reqs_.erase(schedules[k1_pic].unloaded_reqs_.begin() + i);
                break;
            }
        }

        for (uint i = 0; i < schedules[k1_del].reloaded_reqs_.size(); ++i) {
            if (schedules[k1_del].reloaded_reqs_[i].ind_ == req_1) {
                schedules[k1_del].reloaded_reqs_.erase(schedules[k1_del].reloaded_reqs_.begin() + i);
                break;
            }
        }
    }

    if (k2_pic != k2_del) {
        schedules[k2_pic].unloaded_reqs_.insert(schedules[k2_pic].unloaded_reqs_.begin() + unl_pos, UnloadUnit(req_1));
        schedules[k2_del].reloaded_reqs_.push_back(ReloadUnit(req_1));
    }
    solution.schedule_manager_.pic_del_[req_1].first = k2_pic;
    solution.schedule_manager_.pic_del_[req_1].second = k2_del;

    schedules[k1_pic].start_time_ = solution.routes_pickup_[k1_pic].cost_;
    schedules[k2_pic].start_time_ = solution.routes_pickup_[k2_pic].cost_;

    double aux_start_unload;
    set<int> impacted_reloads;
    impacted_reloads.insert(k1_pic);
    impacted_reloads.insert(k2_pic);
    impacted_reloads.insert(k1_del);
    impacted_reloads.insert(k2_del);

    aux_start_unload = schedules[k1_pic].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k1_pic].unloaded_reqs_.size(); ++i) {
        schedules[k1_pic].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[k1_pic].unloaded_reqs_[i].ind_) {

                schedules[solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[k1_pic].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[k1_pic].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(k1_pic, solution.schedule_manager_.pic_del_[schedules[k1_pic].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[k1_pic].unloaded_reqs_.empty()) {
        schedules[k1_pic].unload_end_ = schedules[k1_pic].start_time_;
    } else {
        schedules[k1_pic].unload_end_ = aux_start_unload;
    }

    aux_start_unload = schedules[k2_pic].start_time_ + Instance::instance()->unload_preparation_time();
    for (uint i = 0; i < schedules[k2_pic].unloaded_reqs_.size(); ++i) {
        schedules[k2_pic].unloaded_reqs_[i].start_time_ = aux_start_unload;
        aux_start_unload += Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_);
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second);

        for (uint j = 0; j < schedules[solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_.size(); ++j) {
            if (schedules[solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].ind_
                == schedules[k2_pic].unloaded_reqs_[i].ind_) {

                schedules[solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second].reloaded_reqs_[j].availability_time_ =
                        schedules[k2_pic].unloaded_reqs_[i].start_time_
                        + Instance::instance()->unloading_time(schedules[k2_pic].unloaded_reqs_[i].ind_)
                        + Instance::instance()->between_docks_time(k2_pic, solution.schedule_manager_.pic_del_[schedules[k2_pic].unloaded_reqs_[i].ind_].second);
            }
        }
    }
    if (schedules[k2_pic].unloaded_reqs_.empty()) {
        schedules[k2_pic].unload_end_ = schedules[k2_pic].start_time_;
    } else {
        schedules[k2_pic].unload_end_ = aux_start_unload;
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

bool LocalSearches::ER_General_exchange(Solution & solution, LS_strat search_strat) {

    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    bool improve, general_improvement, is_pickup, best_is_pickup, k1_invert, k2_invert, best_k1_invert, best_k2_invert;
    int k1, pos_i_1, node_i_1;
    int best_pos_i_1 = -1, best_pos_i_2 = -1, best_pos_j_1 = -1, best_pos_j_2 = -1, best_vhc_1 = -1, best_vhc_2 = -1, req_i_1;
    int k1_part1_load, k1_part2_load, k2_part1_load, k2_part2_load;
    int ini_pos_j_2;
    double best_cost, temp_cost, current_cost;
    vector<bool> nodes_already_selected(2 * num_requests + 1);
    map<int, vector<int>> vhc_unload_order, best_vhc_unload_order;

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

            node_i_1 = *selected_node;
            nodes_already_selected[node_i_1] = true;
            is_pickup = node_i_1 <= num_requests ? true : false;
            req_i_1 = is_pickup ? node_i_1 : node_i_1 - num_requests;
            k1 = is_pickup ? solution.schedule_manager_.pic_del_[req_i_1].first : solution.schedule_manager_.pic_del_[req_i_1].second;

            Routes & routes = is_pickup ? solution.routes_pickup_ : solution.routes_delivery_;

            pos_i_1 = find(routes[k1].visited_nodes_.begin(), routes[k1].visited_nodes_.end(), node_i_1) - routes[k1].visited_nodes_.begin();

            temp_cost = solution.cost_;

            k1_part1_load = 0;
            k1_part2_load = routes[k1].load_;

            for (int pos_i_2 = pos_i_1; pos_i_2 <= routes[k1].num_nodes_; ++pos_i_2) {
                k1_part1_load += Instance::instance()->node(routes[k1].visited_nodes_[pos_i_2]).demand();
                k1_part2_load -= Instance::instance()->node(routes[k1].visited_nodes_[pos_i_2]).demand();

                ini_pos_j_2 = (pos_i_2 - pos_i_1 > 0 ? 0 : 1);

                for (int k2 = 0; k2 < num_vehicles; ++k2) {
                    if (k1 != k2) {
                        for (int pos_j_1 = 1; pos_j_1 <= routes[k2].num_nodes_; ++pos_j_1) {
                            if (!nodes_already_selected[routes[k2].visited_nodes_[pos_j_1]]) {
                                if (ini_pos_j_2 > 0) {
                                    k2_part1_load = Instance::instance()->node(routes[k2].visited_nodes_[pos_j_1]).demand();
                                    k2_part2_load = routes[k2].load_ - Instance::instance()->node(routes[k2].visited_nodes_[pos_j_1]).demand();
                                } else {
                                    k2_part1_load = 0;
                                    k2_part2_load = routes[k2].load_;
                                }
                                for (int pos_j_2 = pos_j_1 + ini_pos_j_2; pos_j_2 <= routes[k2].num_nodes_; ++pos_j_2) {
                                    k2_part1_load += Instance::instance()->node(routes[k2].visited_nodes_[pos_j_2]).demand();
                                    k2_part2_load -= Instance::instance()->node(routes[k2].visited_nodes_[pos_j_2]).demand();

                                    if (k1_part1_load + k2_part2_load <= vhc_capacity && k1_part2_load + k2_part1_load <= vhc_capacity) {

                                        vhc_unload_order.clear();
                                        current_cost = temp_cost + (is_pickup ? ER_general_exchange_pickup_delta_cost(solution, k1, pos_i_1, pos_i_2, k2, pos_j_1, pos_j_2, vhc_unload_order, k1_invert, k2_invert)
                                                                    : ER_general_exchange_delivery_delta_cost(solution, k1, pos_i_1, pos_i_2, k2, pos_j_1, pos_j_2, vhc_unload_order, k1_invert, k2_invert));

                                        if (cmp(current_cost, best_cost) < 0) {
                                            best_cost = current_cost;
                                            best_vhc_1 = k1;
                                            best_vhc_2 = k2;
                                            best_pos_i_1 = pos_i_1;
                                            best_pos_i_2 = pos_i_2;
                                            best_pos_j_1 = pos_j_1;
                                            best_pos_j_2 = pos_j_2;
                                            best_is_pickup = is_pickup;
                                            best_k1_invert = k1_invert;
                                            best_k2_invert = k2_invert;
                                            best_vhc_unload_order = vhc_unload_order;
                                            improve = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                    general_improvement = true;
                    if (best_is_pickup) {
                        ER_move_general_exchange_pickup_routes(solution, best_vhc_1, best_pos_i_1, best_pos_i_2, best_vhc_2, best_pos_j_1, best_pos_j_2, best_vhc_unload_order, best_k1_invert, best_k2_invert);
                    } else {
                        ER_move_general_exchange_delivery_routes(solution, best_vhc_1, best_pos_i_1, best_pos_i_2, best_vhc_2, best_pos_j_1, best_pos_j_2, best_vhc_unload_order, best_k1_invert, best_k2_invert);
                    }

                    if (search_strat == LS_strat::complete_first_improv_) {
                        shuffle(selectable_nodes.begin(), selectable_nodes.end(), rand_utils::generator);
                        improve = false;
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
                ER_move_general_exchange_pickup_routes(solution, best_vhc_1, best_pos_i_1, best_pos_i_2, best_vhc_2, best_pos_j_1, best_pos_j_2, best_vhc_unload_order, best_k1_invert, best_k2_invert);
            } else {
                ER_move_general_exchange_delivery_routes(solution, best_vhc_1, best_pos_i_1, best_pos_i_2, best_vhc_2, best_pos_j_1, best_pos_j_2, best_vhc_unload_order, best_k1_invert, best_k2_invert);
            }

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }
    }

    return general_improvement;
}

double LocalSearches::ER_general_exchange_pickup_delta_cost(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                            map<int, vector<int>> & vhc_unload_order, bool & k1_invert, bool & k2_invert) {
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

    // mounting vhc_1
    for (int i = 0; i < pos_i_1 - 1; ++i) {
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i], solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
        reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
    }
    reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1]);
    for (int j = pos_j_1; j < pos_j_2; ++j) {
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[j], solution.routes_pickup_[vhc_2].visited_nodes_[j+1]);
        reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[j+1]);
    }
    for (int i = pos_i_2 + 1; i <= solution.routes_pickup_[vhc_1].num_nodes_; ++i) {
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i], solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
        reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_1].visited_nodes_[i]);
    }
    if (cmp(Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1])
            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2 + 1]),
            Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2])
            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2 + 1])) <= 0) {

        k1_invert = false;
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1])
                            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2 + 1]);
    } else {
        k1_invert = true;
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2])
                            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2 + 1]);
    }

    // mounting vhc_2
    for (int j = 0; j < pos_j_1 - 1; ++j) {
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[j], solution.routes_pickup_[vhc_2].visited_nodes_[j+1]);
        reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_2].visited_nodes_[j+1]);
    }
    reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1]);
    for (int i = pos_i_1; i < pos_i_2; ++i) {
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[i], solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
        reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_1].visited_nodes_[i+1]);
    }
    for (int j = pos_j_2 + 1; j <= solution.routes_pickup_[vhc_2].num_nodes_; ++j) {
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[j], solution.routes_pickup_[vhc_2].visited_nodes_[j+1]);
        reqs_in_vhc_2.insert(solution.routes_pickup_[vhc_2].visited_nodes_[j]);
    }
    if (cmp(Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1])
            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2 + 1]),
            Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2])
            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2 + 1])) <= 0) {

        k2_invert = false;
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1])
                            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2 + 1]);
    } else {
        k2_invert = true;
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2])
                            + Instance::instance()->EdgeCost(solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1], solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2 + 1]);
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
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
                    }
                } else if (reqs_in_vhc_2.find(rs[i].ind_) != reqs_in_vhc_2.end()) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
                    }
                } else {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
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
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_1 && solution.schedule_manager_.pic_del_[*it].second == vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
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
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
                    }
                } else if (reqs_in_vhc_2.find(rs[i].ind_) != reqs_in_vhc_2.end()) {
                    if (solution.schedule_manager_.pic_del_[rs[i].ind_].second == vhc_2) { // don't reload anymore
                        // remover posicao i
                        rs.erase(rs.begin() + i);
                        --i;
                    } else { // changed pickup vhc, but delivery remains the same
                        // muda tempo de deslocamento
                        rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
                    }
                } else {
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
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
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, vhc_2);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_1.begin(); it != reqs_in_vhc_1.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_1 && solution.schedule_manager_.pic_del_[*it].second == vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, vhc_1);
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

double LocalSearches::ER_general_exchange_delivery_delta_cost(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                              map<int, vector<int>> & vhc_unload_order, bool & k1_invert, bool & k2_invert) {
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

    // mounting vhc_1
    for (int i = 0; i < pos_i_1 - 1; ++i) {
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i], solution.routes_delivery_[vhc_1].visited_nodes_[i+1]);
        reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_1].visited_nodes_[i+1] - num_reqs);
    }
    reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1] - num_reqs);
    aux_pic_del[solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1] - num_reqs].second = vhc_1;
    for (int j = pos_j_1; j < pos_j_2; ++j) {
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[j], solution.routes_delivery_[vhc_2].visited_nodes_[j+1]);
        reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[j+1] - num_reqs);
        aux_pic_del[solution.routes_delivery_[vhc_2].visited_nodes_[j+1] - num_reqs].second = vhc_1;
    }
    for (int i = pos_i_2 + 1; i <= solution.routes_delivery_[vhc_1].num_nodes_; ++i) {
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i], solution.routes_delivery_[vhc_1].visited_nodes_[i+1]);
        reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_1].visited_nodes_[i] - num_reqs);
    }
    if (cmp(Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1])
            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2 + 1]),
            Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2])
            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2 + 1])) <= 0) {

        k1_invert = false;
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1])
                            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2 + 1]);
    } else {
        k1_invert = true;
        vhc_1_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1 - 1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2])
                            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2 + 1]);
    }

    // mounting vhc_2
    for (int j = 0; j < pos_j_1 - 1; ++j) {
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[j], solution.routes_delivery_[vhc_2].visited_nodes_[j+1]);
        reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_2].visited_nodes_[j+1] - num_reqs);
    }
    reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1] - num_reqs);
    aux_pic_del[solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1] - num_reqs].second = vhc_2;
    for (int i = pos_i_1; i < pos_i_2; ++i) {
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[i], solution.routes_delivery_[vhc_1].visited_nodes_[i+1]);
        reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_1].visited_nodes_[i+1] - num_reqs);
        aux_pic_del[solution.routes_delivery_[vhc_1].visited_nodes_[i+1] - num_reqs].second = vhc_2;
    }
    for (int j = pos_j_2 + 1; j <= solution.routes_delivery_[vhc_2].num_nodes_; ++j) {
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[j], solution.routes_delivery_[vhc_2].visited_nodes_[j+1]);
        reqs_in_vhc_2.insert(solution.routes_delivery_[vhc_2].visited_nodes_[j] - num_reqs);
    }
    if (cmp(Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1])
            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2 + 1]),
            Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2])
            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2 + 1])) <= 0) {

        k2_invert = false;
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1])
                            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2 + 1]);
    } else {
        k2_invert = true;
        vhc_2_route_cost += Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1 - 1], solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2])
                            + Instance::instance()->EdgeCost(solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1], solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2 + 1]);
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
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
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
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[*it].first);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_2 && solution.schedule_manager_.pic_del_[*it].second != vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[*it].first);
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
                    rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[rs[i].ind_].first);
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
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[*it].first);
                }
            }
        } else if (*rld_vhc == vhc_2) {
            curr_unload_end = vhc_2_unload_end;
            for (auto it = reqs_in_vhc_2.begin(); it != reqs_in_vhc_2.end(); ++it) {
                if (solution.schedule_manager_.pic_del_[*it].first != vhc_2 && solution.schedule_manager_.pic_del_[*it].second != vhc_2) {
                    rs.resize(rs.size() + 1);
                    rs[rs.size() - 1].ind_ = *it;
                    rs[rs.size() - 1].availability_time_ = req_start_unload[*it] + Instance::instance()->unloading_time(*it) + Instance::instance()->between_docks_time(*rld_vhc, solution.schedule_manager_.pic_del_[*it].first);
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

void LocalSearches::ER_move_general_exchange_pickup_routes(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                           map<int, vector<int>> & vhc_unload_order, const bool k1_invert, const bool k2_invert) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    set<int> new_reqs_in_vhc_1, new_reqs_in_vhc_2;

    Route route_vhc_1_bkp = solution.routes_pickup_[vhc_1];

    int stop_req = solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_2 + 1];
    int insert_pos;
    while (solution.routes_pickup_[vhc_1].visited_nodes_[pos_i_1] != stop_req) {
        solution.routes_pickup_[vhc_1].RemoveVisitedNodeFromPos(pos_i_1);
    }
    insert_pos = pos_i_1;
    for (int j = pos_j_1; j <= pos_j_2; ++j) {
        new_reqs_in_vhc_1.insert(solution.routes_pickup_[vhc_2].visited_nodes_[j]);
        solution.schedule_manager_.pic_del_[solution.routes_pickup_[vhc_2].visited_nodes_[j]].first = vhc_1;
        solution.routes_pickup_[vhc_1].InsertVisitedNode(insert_pos, solution.routes_pickup_[vhc_2].visited_nodes_[j]);
        if (!k1_invert) {
            ++insert_pos;
        }
    }

    stop_req = solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_2 + 1];
    while (solution.routes_pickup_[vhc_2].visited_nodes_[pos_j_1] != stop_req) {
        solution.routes_pickup_[vhc_2].RemoveVisitedNodeFromPos(pos_j_1);
    }
    insert_pos = pos_j_1;
    for (int i = pos_i_1; i <= pos_i_2; ++i) {
        new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i]);
        solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i]].first = vhc_2;
        solution.routes_pickup_[vhc_2].InsertVisitedNode(insert_pos, route_vhc_1_bkp.visited_nodes_[i]);
        if (!k2_invert) {
            ++insert_pos;
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

void LocalSearches::ER_move_general_exchange_delivery_routes(Solution & solution, const int vhc_1, const int pos_i_1, const int pos_i_2, const int vhc_2, const int pos_j_1, const int pos_j_2,
                                                             map<int, vector<int>> & vhc_unload_order, const bool k1_invert, const bool k2_invert) {
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    set<int> new_reqs_in_vhc_1, new_reqs_in_vhc_2;

    Route route_vhc_1_bkp = solution.routes_delivery_[vhc_1];

    int stop_node = solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_2 + 1];
    int insert_pos;
    while (solution.routes_delivery_[vhc_1].visited_nodes_[pos_i_1] != stop_node) {
        solution.routes_delivery_[vhc_1].RemoveVisitedNodeFromPos(pos_i_1);
    }
    insert_pos = pos_i_1;
    for (int j = pos_j_1; j <= pos_j_2; ++j) {
        new_reqs_in_vhc_1.insert(solution.routes_delivery_[vhc_2].visited_nodes_[j] - num_reqs);
        solution.schedule_manager_.pic_del_[solution.routes_delivery_[vhc_2].visited_nodes_[j] - num_reqs].second = vhc_1;
        solution.routes_delivery_[vhc_1].InsertVisitedNode(insert_pos, solution.routes_delivery_[vhc_2].visited_nodes_[j]);
        if (!k1_invert) {
            ++insert_pos;
        }
    }

    stop_node = solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_2 + 1];
    while (solution.routes_delivery_[vhc_2].visited_nodes_[pos_j_1] != stop_node) {
        solution.routes_delivery_[vhc_2].RemoveVisitedNodeFromPos(pos_j_1);
    }
    insert_pos = pos_j_1;
    for (int i = pos_i_1; i <= pos_i_2; ++i) {
        new_reqs_in_vhc_2.insert(route_vhc_1_bkp.visited_nodes_[i] - num_reqs);
        solution.schedule_manager_.pic_del_[route_vhc_1_bkp.visited_nodes_[i] - num_reqs].second = vhc_2;
        solution.routes_delivery_[vhc_2].InsertVisitedNode(insert_pos, route_vhc_1_bkp.visited_nodes_[i]);
        if (!k2_invert) {
            ++insert_pos;
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
