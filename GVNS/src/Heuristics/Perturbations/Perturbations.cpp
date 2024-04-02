#include "Perturbations.h"

Perturbations::Perturbations() {
    edge_penalties_ = vector<vector<int>>(Instance::instance()->num_nodes(), vector<int>(Instance::instance()->num_nodes(), 0));
    load_penalties_ = vector<int>(Instance::instance()->num_requests() + 1, 1);
    lambda_ = 0.5;
}

void Perturbations::IR_Multiple_exchanges(Solution & solution) {
    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int num_route_nodes, num_moves, rnd_aux;
    int pos_node_i, pos_node_j;

    Solution solution_aux(num_requests, num_vehicles);
    solution_aux.routes_pickup_ = solution.routes_pickup_;
    solution_aux.routes_delivery_ = solution.routes_delivery_;

    for (int pd = 0; pd < 2; ++pd) {
        Routes & routes = (pd == 0 ? solution_aux.routes_pickup_ : solution_aux.routes_delivery_);
        Routes & orig_routes = (pd == 0 ? solution.routes_pickup_ : solution.routes_delivery_);

        for (int k = 0; k < num_vehicles; ++k) {
            num_route_nodes = routes[k].num_nodes_;

            if (num_route_nodes > 3) {
                num_moves = uni_rand(1, num_route_nodes - 1);
                for (int c = 0; c < num_moves; ++c) {
                    pos_node_i = uni_rand(1, num_route_nodes);

                    do {
                        pos_node_j = uni_rand(1, num_route_nodes);
                    } while(pos_node_j == pos_node_i
                            || routes[k].visited_nodes_[pos_node_j] == orig_routes[k].visited_nodes_[pos_node_i]
                            || routes[k].visited_nodes_[pos_node_i] == orig_routes[k].visited_nodes_[pos_node_j]);

                    swap(routes[k].visited_nodes_[pos_node_i], routes[k].visited_nodes_[pos_node_j]);
                }
            } else if (num_route_nodes == 3) {
                rnd_aux = uni_rand(1,2);

                if (rnd_aux == 1) {
                    swap(routes[k].visited_nodes_[1], routes[k].visited_nodes_[2]);
                } else {
                    swap(routes[k].visited_nodes_[2], routes[k].visited_nodes_[3]);
                }
            }

            if (num_route_nodes >= 3) {
                routes[k].cost_ = 0.0;
                for (int i = 0; i <= routes[k].num_nodes_; ++i) {
                    routes[k].cost_ += Instance::instance()->EdgeCost(routes[k].visited_nodes_[i], routes[k].visited_nodes_[i + 1]);
                }
            }
        }
    }

    ConstructiveHeuristics ch;
    ch.DefineScheduling2(solution_aux);

    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution_aux.cost_ += solution_aux.routes_delivery_[k].cost_ + solution_aux.schedule_manager_.schedules_[k].completion_time_;
    }

    solution = solution_aux;
}

void Perturbations::UN_Multiple_exchanges(Solution & solution) {
    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int num_unl_reqs, num_moves;
    int pos_node_i, pos_node_j;

    Solution solution_aux = solution;

    Schedules & schedules = solution_aux.schedule_manager_.schedules_;
    Schedules & orig_schedules = solution.schedule_manager_.schedules_;

    for (int k = 0; k < num_vehicles; ++k) {
        if (schedules[k].unloaded_reqs_.size() > 1) {
            num_unl_reqs = static_cast<int>(schedules[k].unloaded_reqs_.size());
            num_moves = uni_rand(1, num_unl_reqs - 1);

            for (int c = 0; c < num_moves; ++c) {
                pos_node_i = uni_rand(0, num_unl_reqs - 1);

                do {
                    pos_node_j = uni_rand(0, num_unl_reqs - 1);
                } while(pos_node_j == pos_node_i
                        || schedules[k].unloaded_reqs_[pos_node_j].ind_ == orig_schedules[k].unloaded_reqs_[pos_node_i].ind_
                        || schedules[k].unloaded_reqs_[pos_node_i].ind_ == orig_schedules[k].unloaded_reqs_[pos_node_j].ind_);

                swap(schedules[k].unloaded_reqs_[pos_node_i], schedules[k].unloaded_reqs_[pos_node_j]);
            }
        }
    }

    vector<double> req_start_unload(num_requests + 1, -1.0);
    double aux_start_unload;
    for (int k = 0; k < num_vehicles; ++k) {
        aux_start_unload = schedules[k].start_time_ + Instance::instance()->unload_preparation_time();
        for (uint i = 0; i < schedules[k].unloaded_reqs_.size(); ++i) {
            schedules[k].unloaded_reqs_[i].start_time_ = aux_start_unload;
            req_start_unload[schedules[k].unloaded_reqs_[i].ind_] = aux_start_unload;
            aux_start_unload += Instance::instance()->unloading_time(schedules[k].unloaded_reqs_[i].ind_);
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
            schedules[k].reloaded_reqs_[i].availability_time_ = req_start_unload[schedules[k].reloaded_reqs_[i].ind_]
                                                                + Instance::instance()->unloading_time(schedules[k].reloaded_reqs_[i].ind_)
                                                                + Instance::instance()->between_docks_time(solution_aux.schedule_manager_.pic_del_[schedules[k].reloaded_reqs_[i].ind_].first, k);
        }
    }

    double infeas_st_av_size;
    for (int k = 0; k < num_vehicles; ++k) {
        if (!schedules[k].reloaded_reqs_.empty()) {
            sort(schedules[k].reloaded_reqs_.begin(), schedules[k].reloaded_reqs_.end());
            schedules[k].reloaded_reqs_[0].start_time_ = schedules[k].unload_end_ + Instance::instance()->reload_preparation_time();

            infeas_st_av_size = schedules[k].reloaded_reqs_[0].start_time_ - schedules[k].reloaded_reqs_[0].availability_time_;
            for (uint i = 1; i < schedules[k].reloaded_reqs_.size(); ++i) {
                schedules[k].reloaded_reqs_[i].start_time_ = schedules[k].reloaded_reqs_[i-1].start_time_
                                                             + Instance::instance()->reloading_time(schedules[k].reloaded_reqs_[i-1].ind_);

                if (cmp(infeas_st_av_size, schedules[k].reloaded_reqs_[i].start_time_ - schedules[k].reloaded_reqs_[i].availability_time_) > 0) {
                    infeas_st_av_size = schedules[k].reloaded_reqs_[i].start_time_ - schedules[k].reloaded_reqs_[i].availability_time_;
                }
            }

            if (cmp(infeas_st_av_size, 0.0) < 0) {
                for (uint i = 0; i < schedules[k].reloaded_reqs_.size(); ++i) {
                    schedules[k].reloaded_reqs_[i].start_time_ -= infeas_st_av_size;
                }
            }

            schedules[k].completion_time_ = schedules[k].reloaded_reqs_.back().start_time_
                                            + Instance::instance()->reloading_time(schedules[k].reloaded_reqs_.back().ind_);
        } else {
            schedules[k].completion_time_ = schedules[k].unload_end_;
        }
    }

    solution_aux.cost_ = 0.0;
    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution_aux.cost_ += solution_aux.routes_delivery_[k].cost_ + solution_aux.schedule_manager_.schedules_[k].completion_time_;
    }

    solution = solution_aux;
}

void Perturbations::ER_Alignment(Solution & solution) {
    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    vector<int> reqs_with_cd_operation;
    int pd, pic_vhc, del_vhc, curr_demand, exchangeable_node;
    set<int> already_used_reqs;
    bool move_done;

    Solution solution_aux(num_requests, num_vehicles);
    solution_aux.routes_pickup_ = solution.routes_pickup_;
    solution_aux.routes_delivery_ = solution.routes_delivery_;

    for (int r = 1; r <= num_requests; ++r) {
        if (solution.schedule_manager_.pic_del_[r].first != solution.schedule_manager_.pic_del_[r].second) {
            reqs_with_cd_operation.push_back(r);
        }
    }

    Routes & pic_routes = solution_aux.routes_pickup_;
    Routes & del_routes = solution_aux.routes_delivery_;

    shuffle(reqs_with_cd_operation.begin(), reqs_with_cd_operation.end(), rand_utils::generator);

    for (int curr_req : reqs_with_cd_operation) {
        if (already_used_reqs.find(curr_req) == already_used_reqs.end()) {
            pic_vhc = solution.schedule_manager_.pic_del_[curr_req].first;
            del_vhc = solution.schedule_manager_.pic_del_[curr_req].second;
            curr_demand = Instance::instance()->node(curr_req).demand();

            pd = uni_rand(0,1);
            move_done = false;

            if (pd == 0) {
                if (pic_routes[pic_vhc].num_nodes_ > 1) {
                    if (curr_demand <= vhc_capacity - pic_routes[del_vhc].load_) {
                        for (int i = 1; i <= pic_routes[pic_vhc].num_nodes_; ++i) {
                            if (pic_routes[pic_vhc].visited_nodes_[i] == curr_req) {
                                pic_routes[pic_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        pic_routes[del_vhc].InsertVisitedNode(uni_rand(1, pic_routes[del_vhc].last_insertable_pos_), curr_req);
                        move_done = true;
                    } else {
                        exchangeable_node = -1;
                        vector<int> route_inds(pic_routes[del_vhc].num_nodes_);
                        iota(route_inds.begin(), route_inds.end(), 1);
                        shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                        for (int i : route_inds) {
                            if (Instance::instance()->node(pic_routes[del_vhc].visited_nodes_[i]).demand() <= vhc_capacity - pic_routes[pic_vhc].load_ + curr_demand
                                && curr_demand <= vhc_capacity - pic_routes[del_vhc].load_ + Instance::instance()->node(pic_routes[del_vhc].visited_nodes_[i]).demand()) {

                                exchangeable_node = pic_routes[del_vhc].visited_nodes_[i];
                                pic_routes[del_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        if (exchangeable_node > 0) {
                            for (int i = 1; i <= pic_routes[pic_vhc].num_nodes_; ++i) {
                                if (pic_routes[pic_vhc].visited_nodes_[i] == curr_req) {
                                    pic_routes[pic_vhc].RemoveVisitedNodeFromPos(i);
                                    break;
                                }
                            }

                            pic_routes[del_vhc].InsertVisitedNode(uni_rand(1, pic_routes[del_vhc].last_insertable_pos_), curr_req);
                            pic_routes[pic_vhc].InsertVisitedNode(uni_rand(1, pic_routes[pic_vhc].last_insertable_pos_), exchangeable_node);

                            move_done = true;
                            already_used_reqs.insert(exchangeable_node);
                        }
                    }
                }

                if (!move_done && del_routes[del_vhc].num_nodes_ > 1) {
                    if (curr_demand <= vhc_capacity - del_routes[pic_vhc].load_) {
                        for (int i = 1; i <= del_routes[del_vhc].num_nodes_; ++i) {
                            if (del_routes[del_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                del_routes[del_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        del_routes[pic_vhc].InsertVisitedNode(uni_rand(1, del_routes[pic_vhc].last_insertable_pos_), curr_req + num_requests);
                    } else {
                        exchangeable_node = -1;
                        vector<int> route_inds(del_routes[pic_vhc].num_nodes_);
                        iota(route_inds.begin(), route_inds.end(), 1);
                        shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                        for (int i : route_inds) {
                            if (Instance::instance()->node(del_routes[pic_vhc].visited_nodes_[i]).demand() <= vhc_capacity - del_routes[del_vhc].load_ + curr_demand
                                && curr_demand <= vhc_capacity - del_routes[pic_vhc].load_ + Instance::instance()->node(del_routes[pic_vhc].visited_nodes_[i]).demand()) {

                                exchangeable_node = del_routes[pic_vhc].visited_nodes_[i];
                                del_routes[pic_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        if (exchangeable_node > 0) {
                            for (int i = 1; i <= del_routes[del_vhc].num_nodes_; ++i) {
                                if (del_routes[del_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                    del_routes[del_vhc].RemoveVisitedNodeFromPos(i);
                                    break;
                                }
                            }

                            del_routes[pic_vhc].InsertVisitedNode(uni_rand(1, del_routes[pic_vhc].last_insertable_pos_), curr_req + num_requests);
                            del_routes[del_vhc].InsertVisitedNode(uni_rand(1, del_routes[del_vhc].last_insertable_pos_), exchangeable_node);

                            already_used_reqs.insert(exchangeable_node - num_requests);
                        }
                    }
                }
            } else {
                if (del_routes[del_vhc].num_nodes_ > 1) {
                    if (curr_demand <= vhc_capacity - del_routes[pic_vhc].load_) {
                        for (int i = 1; i <= del_routes[del_vhc].num_nodes_; ++i) {
                            if (del_routes[del_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                del_routes[del_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        del_routes[pic_vhc].InsertVisitedNode(uni_rand(1, del_routes[pic_vhc].last_insertable_pos_), curr_req + num_requests);
                        move_done = true;
                    } else {
                        exchangeable_node = -1;
                        vector<int> route_inds(del_routes[pic_vhc].num_nodes_);
                        iota(route_inds.begin(), route_inds.end(), 1);
                        shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                        for (int i : route_inds) {
                            if (Instance::instance()->node(del_routes[pic_vhc].visited_nodes_[i]).demand() <= vhc_capacity - del_routes[del_vhc].load_ + curr_demand
                                && curr_demand <= vhc_capacity - del_routes[pic_vhc].load_ + Instance::instance()->node(del_routes[pic_vhc].visited_nodes_[i]).demand()) {

                                exchangeable_node = del_routes[pic_vhc].visited_nodes_[i];
                                del_routes[pic_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        if (exchangeable_node > 0) {
                            for (int i = 1; i <= del_routes[del_vhc].num_nodes_; ++i) {
                                if (del_routes[del_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                    del_routes[del_vhc].RemoveVisitedNodeFromPos(i);
                                    break;
                                }
                            }

                            del_routes[pic_vhc].InsertVisitedNode(uni_rand(1, del_routes[pic_vhc].last_insertable_pos_), curr_req + num_requests);
                            del_routes[del_vhc].InsertVisitedNode(uni_rand(1, del_routes[del_vhc].last_insertable_pos_), exchangeable_node);

                            move_done = true;
                            already_used_reqs.insert(exchangeable_node - num_requests);
                        }
                    }
                }

                if (!move_done && pic_routes[pic_vhc].num_nodes_ > 1) {
                    if (curr_demand <= vhc_capacity - pic_routes[del_vhc].load_) {
                        for (int i = 1; i <= pic_routes[pic_vhc].num_nodes_; ++i) {
                            if (pic_routes[pic_vhc].visited_nodes_[i] == curr_req) {
                                pic_routes[pic_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        pic_routes[del_vhc].InsertVisitedNode(uni_rand(1, pic_routes[del_vhc].last_insertable_pos_), curr_req);
                    } else {
                        exchangeable_node = -1;
                        vector<int> route_inds(pic_routes[del_vhc].num_nodes_);
                        iota(route_inds.begin(), route_inds.end(), 1);
                        shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                        for (int i : route_inds) {
                            if (Instance::instance()->node(pic_routes[del_vhc].visited_nodes_[i]).demand() <= vhc_capacity - pic_routes[pic_vhc].load_ + curr_demand
                                && curr_demand <= vhc_capacity - pic_routes[del_vhc].load_ + Instance::instance()->node(pic_routes[del_vhc].visited_nodes_[i]).demand()) {

                                exchangeable_node = pic_routes[del_vhc].visited_nodes_[i];
                                pic_routes[del_vhc].RemoveVisitedNodeFromPos(i);
                                break;
                            }
                        }

                        if (exchangeable_node > 0) {
                            for (int i = 1; i <= pic_routes[pic_vhc].num_nodes_; ++i) {
                                if (pic_routes[pic_vhc].visited_nodes_[i] == curr_req) {
                                    pic_routes[pic_vhc].RemoveVisitedNodeFromPos(i);
                                    break;
                                }
                            }

                            pic_routes[del_vhc].InsertVisitedNode(uni_rand(1, pic_routes[del_vhc].last_insertable_pos_), curr_req);
                            pic_routes[pic_vhc].InsertVisitedNode(uni_rand(1, pic_routes[pic_vhc].last_insertable_pos_), exchangeable_node);

                            already_used_reqs.insert(exchangeable_node);
                        }
                    }
                }
            }
        }
    }

    ConstructiveHeuristics ch;
    ch.DefineScheduling2(solution_aux);

    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution_aux.cost_ += solution_aux.routes_delivery_[k].cost_ + solution_aux.schedule_manager_.schedules_[k].completion_time_;
    }

    solution = solution_aux;
}

void Perturbations::ER_Misalignment(Solution & solution) {
    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    vector<int> reqs_without_cd_operation;
    int pd, curr_vhc, curr_demand, exchangeable_node;
    set<int> already_used_reqs;
    bool move_done;
    vector<int> vhc_inds(num_vehicles);
    iota(vhc_inds.begin(), vhc_inds.end(), 0);

    Solution solution_aux(num_requests, num_vehicles);
    solution_aux.routes_pickup_ = solution.routes_pickup_;
    solution_aux.routes_delivery_ = solution.routes_delivery_;

    for (int r = 1; r <= num_requests; ++r) {
        if (solution.schedule_manager_.pic_del_[r].first == solution.schedule_manager_.pic_del_[r].second) {
            reqs_without_cd_operation.push_back(r);
        }
    }

    Routes & pic_routes = solution_aux.routes_pickup_;
    Routes & del_routes = solution_aux.routes_delivery_;

    shuffle(reqs_without_cd_operation.begin(), reqs_without_cd_operation.end(), rand_utils::generator);

    for (int curr_req : reqs_without_cd_operation) {
        if (already_used_reqs.find(curr_req) == already_used_reqs.end()) {
            curr_vhc = solution.schedule_manager_.pic_del_[curr_req].first;
            curr_demand = Instance::instance()->node(curr_req).demand();

            pd = uni_rand(0,1);
            move_done = false;

            if (pd == 0) {
                if (pic_routes[curr_vhc].num_nodes_ > 1) {
                    shuffle(vhc_inds.begin(), vhc_inds.end(), rand_utils::generator);

                    for (int vhc : vhc_inds) {
                        if (vhc != curr_vhc) {
                            if (curr_demand <= vhc_capacity - pic_routes[vhc].load_) {
                                for (int i = 1; i <= pic_routes[curr_vhc].num_nodes_; ++i) {
                                    if (pic_routes[curr_vhc].visited_nodes_[i] == curr_req) {
                                        pic_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                pic_routes[vhc].InsertVisitedNode(uni_rand(1, pic_routes[vhc].last_insertable_pos_), curr_req);
                                move_done = true;
                            } else {
                                exchangeable_node = -1;
                                vector<int> route_inds(pic_routes[vhc].num_nodes_);
                                iota(route_inds.begin(), route_inds.end(), 1);
                                shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                                for (int i : route_inds) {
                                    if (Instance::instance()->node(pic_routes[vhc].visited_nodes_[i]).demand() <= vhc_capacity - pic_routes[curr_vhc].load_ + curr_demand
                                        && curr_demand <= vhc_capacity - pic_routes[vhc].load_ + Instance::instance()->node(pic_routes[vhc].visited_nodes_[i]).demand()) {

                                        exchangeable_node = pic_routes[vhc].visited_nodes_[i];
                                        pic_routes[vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                if (exchangeable_node > 0) {
                                    for (int i = 1; i <= pic_routes[curr_vhc].num_nodes_; ++i) {
                                        if (pic_routes[curr_vhc].visited_nodes_[i] == curr_req) {
                                            pic_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                            break;
                                        }
                                    }

                                    pic_routes[vhc].InsertVisitedNode(uni_rand(1, pic_routes[vhc].last_insertable_pos_), curr_req);
                                    pic_routes[curr_vhc].InsertVisitedNode(uni_rand(1, pic_routes[curr_vhc].last_insertable_pos_), exchangeable_node);

                                    move_done = true;
                                    already_used_reqs.insert(exchangeable_node);
                                }
                            }

                            if (move_done) {
                                break;
                            }
                        }
                    }
                }

                if (!move_done && del_routes[curr_vhc].num_nodes_ > 1) {
                    shuffle(vhc_inds.begin(), vhc_inds.end(), rand_utils::generator);

                    for (int vhc : vhc_inds) {
                        if (vhc != curr_vhc) {
                            if (curr_demand <= vhc_capacity - del_routes[vhc].load_) {
                                for (int i = 1; i <= del_routes[curr_vhc].num_nodes_; ++i) {
                                    if (del_routes[curr_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                        del_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                del_routes[vhc].InsertVisitedNode(uni_rand(1, del_routes[vhc].last_insertable_pos_), curr_req + num_requests);
                                move_done = true;
                            } else {
                                exchangeable_node = -1;
                                vector<int> route_inds(del_routes[vhc].num_nodes_);
                                iota(route_inds.begin(), route_inds.end(), 1);
                                shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                                for (int i : route_inds) {
                                    if (Instance::instance()->node(del_routes[vhc].visited_nodes_[i]).demand() <= vhc_capacity - del_routes[curr_vhc].load_ + curr_demand
                                        && curr_demand <= vhc_capacity - del_routes[vhc].load_ + Instance::instance()->node(del_routes[vhc].visited_nodes_[i]).demand()) {

                                        exchangeable_node = del_routes[vhc].visited_nodes_[i];
                                        del_routes[vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                if (exchangeable_node > 0) {
                                    for (int i = 1; i <= del_routes[curr_vhc].num_nodes_; ++i) {
                                        if (del_routes[curr_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                            del_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                            break;
                                        }
                                    }

                                    del_routes[vhc].InsertVisitedNode(uni_rand(1, del_routes[vhc].last_insertable_pos_), curr_req + num_requests);
                                    del_routes[curr_vhc].InsertVisitedNode(uni_rand(1, del_routes[curr_vhc].last_insertable_pos_), exchangeable_node);

                                    move_done = true;
                                    already_used_reqs.insert(exchangeable_node - num_requests);
                                }
                            }

                            if (move_done) {
                                break;
                            }
                        }
                    }
                }
            } else {
                if (del_routes[curr_vhc].num_nodes_ > 1) {
                    shuffle(vhc_inds.begin(), vhc_inds.end(), rand_utils::generator);

                    for (int vhc : vhc_inds) {
                        if (vhc != curr_vhc) {
                            if (curr_demand <= vhc_capacity - del_routes[vhc].load_) {
                                for (int i = 1; i <= del_routes[curr_vhc].num_nodes_; ++i) {
                                    if (del_routes[curr_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                        del_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                del_routes[vhc].InsertVisitedNode(uni_rand(1, del_routes[vhc].last_insertable_pos_), curr_req + num_requests);
                                move_done = true;
                            } else {
                                exchangeable_node = -1;
                                vector<int> route_inds(del_routes[vhc].num_nodes_);
                                iota(route_inds.begin(), route_inds.end(), 1);
                                shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                                for (int i : route_inds) {
                                    if (Instance::instance()->node(del_routes[vhc].visited_nodes_[i]).demand() <= vhc_capacity - del_routes[curr_vhc].load_ + curr_demand
                                        && curr_demand <= vhc_capacity - del_routes[vhc].load_ + Instance::instance()->node(del_routes[vhc].visited_nodes_[i]).demand()) {

                                        exchangeable_node = del_routes[vhc].visited_nodes_[i];
                                        del_routes[vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                if (exchangeable_node > 0) {
                                    for (int i = 1; i <= del_routes[curr_vhc].num_nodes_; ++i) {
                                        if (del_routes[curr_vhc].visited_nodes_[i] == curr_req + num_requests) {
                                            del_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                            break;
                                        }
                                    }

                                    del_routes[vhc].InsertVisitedNode(uni_rand(1, del_routes[vhc].last_insertable_pos_), curr_req + num_requests);
                                    del_routes[curr_vhc].InsertVisitedNode(uni_rand(1, del_routes[curr_vhc].last_insertable_pos_), exchangeable_node);

                                    move_done = true;
                                    already_used_reqs.insert(exchangeable_node - num_requests);
                                }
                            }

                            if (move_done) {
                                break;
                            }
                        }
                    }
                }

                if (!move_done && pic_routes[curr_vhc].num_nodes_ > 1) {
                    shuffle(vhc_inds.begin(), vhc_inds.end(), rand_utils::generator);

                    for (int vhc : vhc_inds) {
                        if (vhc != curr_vhc) {
                            if (curr_demand <= vhc_capacity - pic_routes[vhc].load_) {
                                for (int i = 1; i <= pic_routes[curr_vhc].num_nodes_; ++i) {
                                    if (pic_routes[curr_vhc].visited_nodes_[i] == curr_req) {
                                        pic_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                pic_routes[vhc].InsertVisitedNode(uni_rand(1, pic_routes[vhc].last_insertable_pos_), curr_req);
                                move_done = true;
                            } else {
                                exchangeable_node = -1;
                                vector<int> route_inds(pic_routes[vhc].num_nodes_);
                                iota(route_inds.begin(), route_inds.end(), 1);
                                shuffle(route_inds.begin(), route_inds.end(), rand_utils::generator);

                                for (int i : route_inds) {
                                    if (Instance::instance()->node(pic_routes[vhc].visited_nodes_[i]).demand() <= vhc_capacity - pic_routes[curr_vhc].load_ + curr_demand
                                        && curr_demand <= vhc_capacity - pic_routes[vhc].load_ + Instance::instance()->node(pic_routes[vhc].visited_nodes_[i]).demand()) {

                                        exchangeable_node = pic_routes[vhc].visited_nodes_[i];
                                        pic_routes[vhc].RemoveVisitedNodeFromPos(i);
                                        break;
                                    }
                                }

                                if (exchangeable_node > 0) {
                                    for (int i = 1; i <= pic_routes[curr_vhc].num_nodes_; ++i) {
                                        if (pic_routes[curr_vhc].visited_nodes_[i] == curr_req) {
                                            pic_routes[curr_vhc].RemoveVisitedNodeFromPos(i);
                                            break;
                                        }
                                    }

                                    pic_routes[vhc].InsertVisitedNode(uni_rand(1, pic_routes[vhc].last_insertable_pos_), curr_req);
                                    pic_routes[curr_vhc].InsertVisitedNode(uni_rand(1, pic_routes[curr_vhc].last_insertable_pos_), exchangeable_node);

                                    move_done = true;
                                    already_used_reqs.insert(exchangeable_node);
                                }
                            }

                            if (move_done) {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    ConstructiveHeuristics ch;
    ch.DefineScheduling2(solution_aux);

    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution_aux.cost_ += solution_aux.routes_delivery_[k].cost_ + solution_aux.schedule_manager_.schedules_[k].completion_time_;
    }

    solution = solution_aux;
}

void Perturbations::ER_Mix_cyclic(Solution & solution) {
    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();

    Solution solution_aux(num_requests, num_vehicles);

    vector<int> available_routes(num_vehicles);
    iota(available_routes.begin(), available_routes.end(), 0);

    vector<vector<int>> hand_1(2), hand_2(2);
    vector<pair<vector<int>, vector<int>>> feasible_hands_2;
    vector<pair<int, int>> pic_del = solution.schedule_manager_.pic_del_;
    vector<int> hand_1_size(2);
    vector<int> load(2);
    int k1, k2;
    int moves, max_moves, rnd_pos_1, rnd_pos_2, move_type, feasible_choosed;

    for (int pd = 0; pd < 2; ++pd) {
        Routes & routes = (pd == 0 ? solution.routes_pickup_ : solution.routes_delivery_);
        shuffle(available_routes.begin(), available_routes.end(), rand_utils::generator);

        vector<vector<int>> all_hands(num_vehicles);
        vector<int> all_loads(num_vehicles);
        for (int k = 0; k < num_vehicles; ++k) {
            all_hands[k] = vector<int>(routes[k].visited_nodes_.begin() + 1, routes[k].visited_nodes_.end() - 1);
            all_loads[k] = routes[k].load_;
        }

        for (int ar = 0; ar < static_cast<int>(available_routes.size()); ++ar) {
            k1 = available_routes[ar];
            k2 = available_routes[(ar + 1) % available_routes.size()];

            hand_1[0] = all_hands[k1];
            hand_1[1] = all_hands[k2];

            hand_1_size[0] = static_cast<int>(hand_1[0].size());
            hand_1_size[1] = static_cast<int>(hand_1[1].size());

            load[0] = all_loads[k1];
            load[1] = all_loads[k2];

            moves = 0;
            max_moves = hand_1_size[0] + hand_1_size[1];

            for (int turn = 0, other = 1; moves < max_moves; turn = (turn + 1) % 2, other = (other + 1) % 2) {
                if (hand_1_size[turn] > 0) {
                    rnd_pos_1 = uni_rand(0, hand_1_size[turn] - 1);

                    if (hand_1_size[other] > 0) {
                        move_type = uni_rand(1,3);
                    } else {
                        move_type = 2;
                    }

                    if (move_type == 1) { // exchange
                        rnd_pos_2 = uni_rand(0, hand_1_size[other] - 1);

                        hand_2[other].push_back(hand_1[turn][rnd_pos_1]);
                        hand_2[turn].push_back(hand_1[other][rnd_pos_2]);

                        load[turn] -= Instance::instance()->node(hand_1[turn][rnd_pos_1]).demand();
                        load[turn] += Instance::instance()->node(hand_1[other][rnd_pos_2]).demand();

                        load[other] -= Instance::instance()->node(hand_1[other][rnd_pos_2]).demand();
                        load[other] += Instance::instance()->node(hand_1[turn][rnd_pos_1]).demand();

                        hand_1[turn][rnd_pos_1] = hand_1[turn][hand_1_size[turn] - 1];
                        hand_1[other][rnd_pos_2] = hand_1[other][hand_1_size[other] - 1];

                        --hand_1_size[turn];
                        --hand_1_size[other];

                        moves += 2;

                    } else if (move_type == 2) { // reinsertion
                        hand_2[other].push_back(hand_1[turn][rnd_pos_1]);

                        load[turn] -= Instance::instance()->node(hand_1[turn][rnd_pos_1]).demand();
                        load[other] += Instance::instance()->node(hand_1[turn][rnd_pos_1]).demand();

                        hand_1[turn][rnd_pos_1] = hand_1[turn][hand_1_size[turn] - 1];

                        --hand_1_size[turn];

                        ++moves;

                    } else {
                        continue;
                    }

                    if (load[turn] > 0 && load[other] > 0 && load[turn] <= vhc_capacity && load[other] <= vhc_capacity) {
                        feasible_hands_2.push_back(make_pair(hand_2[0], hand_2[1]));
                    }
                }
            }

            feasible_choosed = uni_rand(0, feasible_hands_2.size() - 1);

            if (pd == 0) {
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].first.size()); ++i) {
                    pic_del[feasible_hands_2[feasible_choosed].first[i]].first = k1;
                    all_loads[k1] += Instance::instance()->node(feasible_hands_2[feasible_choosed].first[i]).demand();
                    all_loads[k2] -= Instance::instance()->node(feasible_hands_2[feasible_choosed].first[i]).demand();
                }
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].second.size()); ++i) {
                    pic_del[feasible_hands_2[feasible_choosed].second[i]].first = k2;
                    all_loads[k2] += Instance::instance()->node(feasible_hands_2[feasible_choosed].second[i]).demand();
                    all_loads[k1] -= Instance::instance()->node(feasible_hands_2[feasible_choosed].second[i]).demand();
                }

                for (int i = 0; i < static_cast<int>(all_hands[k1].size()); ++i) {
                    if (pic_del[all_hands[k1][i]].first != k1) {
                        all_hands[k1].erase(all_hands[k1].begin() + i);
                        --i;
                    }
                }
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].first.size()); ++i) {
                    all_hands[k1].push_back(feasible_hands_2[feasible_choosed].first[i]);
                }

                for (int i = 0; i < static_cast<int>(all_hands[k2].size()); ++i) {
                    if (pic_del[all_hands[k2][i]].first != k2) {
                        all_hands[k2].erase(all_hands[k2].begin() + i);
                        --i;
                    }
                }
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].second.size()); ++i) {
                    all_hands[k2].push_back(feasible_hands_2[feasible_choosed].second[i]);
                }
            } else {
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].first.size()); ++i) {
                    pic_del[feasible_hands_2[feasible_choosed].first[i] - num_requests].second = k1;
                    all_loads[k1] += Instance::instance()->node(feasible_hands_2[feasible_choosed].first[i]).demand();
                    all_loads[k2] -= Instance::instance()->node(feasible_hands_2[feasible_choosed].first[i]).demand();
                }
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].second.size()); ++i) {
                    pic_del[feasible_hands_2[feasible_choosed].second[i] - num_requests].second = k2;
                    all_loads[k2] += Instance::instance()->node(feasible_hands_2[feasible_choosed].second[i]).demand();
                    all_loads[k1] -= Instance::instance()->node(feasible_hands_2[feasible_choosed].second[i]).demand();
                }

                for (int i = 0; i < static_cast<int>(all_hands[k1].size()); ++i) {
                    if (pic_del[all_hands[k1][i] - num_requests].second != k1) {
                        all_hands[k1].erase(all_hands[k1].begin() + i);
                        --i;
                    }
                }
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].first.size()); ++i) {
                    all_hands[k1].push_back(feasible_hands_2[feasible_choosed].first[i]);
                }

                for (int i = 0; i < static_cast<int>(all_hands[k2].size()); ++i) {
                    if (pic_del[all_hands[k2][i] - num_requests].second != k2) {
                        all_hands[k2].erase(all_hands[k2].begin() + i);
                        --i;
                    }
                }
                for (int i = 0; i < static_cast<int>(feasible_hands_2[feasible_choosed].second.size()); ++i) {
                    all_hands[k2].push_back(feasible_hands_2[feasible_choosed].second[i]);
                }
            }

            feasible_hands_2.clear();
            hand_2[0].clear();
            hand_2[1].clear();
        }

        if (pd == 0) {
            for (int k = 0; k < num_vehicles; ++k) {
                for (int i = 0; i < static_cast<int>(all_hands[k].size()); ++i) {
                    solution_aux.routes_pickup_[k].InsertVisitedNode(solution_aux.routes_pickup_[k].last_insertable_pos_, all_hands[k][i]);
                }
            }
        } else {
            for (int k = 0; k < num_vehicles; ++k) {
                for (int i = 0; i < static_cast<int>(all_hands[k].size()); ++i) {
                    solution_aux.routes_delivery_[k].InsertVisitedNode(solution_aux.routes_delivery_[k].last_insertable_pos_, all_hands[k][i]);
                }
            }
        }

    }

    ConstructiveHeuristics ch;
    ch.DefineScheduling2(solution_aux);

    for (int k = 0; k < Instance::instance()->num_vehicles(); ++k) {
        solution_aux.cost_ += solution_aux.routes_delivery_[k].cost_ + solution_aux.schedule_manager_.schedules_[k].completion_time_;
    }

    solution = solution_aux;
}
