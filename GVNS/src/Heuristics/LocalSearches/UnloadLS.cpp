#include "LocalSearches.h"

bool LocalSearches::UN_1opt(Solution & solution, LS_strat search_strat) {
    int num_vehicles = Instance::instance()->num_vehicles();
    bool improve, general_improvement;
    int num_unloaded_reqs, curr_pos = -1, curr_vhc, best_vhc, best_orig_pos, best_dest_pos;
    double best_cost, temp_cost, current_cost;
    vector<int> selectable_reqs;
    bool best_going_forward = false;
    int best_unload_size = -1;

    for (int k = 0; k < num_vehicles; ++k) {
        for (unsigned int i = 0; i < solution.schedule_manager_.schedules_[k].unloaded_reqs_.size(); ++i) {
            selectable_reqs.push_back(solution.schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_);
        }
    }
    shuffle(selectable_reqs.begin(), selectable_reqs.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;
    while (improve) {
        improve = false;
        best_cost = solution.cost_;

        for (auto selected_req = selectable_reqs.begin(); selected_req != selectable_reqs.end(); ++selected_req) {
            curr_vhc = solution.schedule_manager_.pic_del_[*selected_req].first;
            num_unloaded_reqs = solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_.size();
            for (int i = 0; i < num_unloaded_reqs; ++i) {
                if (solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_[i].ind_ == *selected_req) {
                    curr_pos = i;
                    break;
                }
            }

            temp_cost = solution.cost_;

            for (int i = 0; i <= num_unloaded_reqs; ++i) {
                if (i < curr_pos || i > curr_pos + 1) {
                    current_cost = temp_cost + UN_1opt_delta_cost(solution, curr_vhc, i, curr_pos);

                    if (cmp(current_cost, best_cost) < 0) {
                        best_cost = current_cost;
                        best_orig_pos = curr_pos;
                        best_dest_pos = i;
                        best_vhc = curr_vhc;
                        best_going_forward = i > curr_pos;
                        best_unload_size = Instance::instance()->unloading_time(solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_[curr_pos].ind_);
                        improve = true;
                    } else if (cmp(current_cost, best_cost) == 0
                               && ((i > curr_pos && best_going_forward
                                    && Instance::instance()->unloading_time(solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_[curr_pos].ind_) > best_unload_size)
                                   || (i < curr_pos && !best_going_forward
                                       && Instance::instance()->unloading_time(solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_[curr_pos].ind_) < best_unload_size))) {
                        best_cost = current_cost;
                        best_orig_pos = curr_pos;
                        best_dest_pos = i;
                        best_vhc = curr_vhc;
                        best_going_forward = i > curr_pos;
                        best_unload_size = Instance::instance()->unloading_time(solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_[curr_pos].ind_);
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;
                UN_move_1opt(solution, best_vhc, best_orig_pos, best_dest_pos);
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
            UN_move_1opt(solution, best_vhc, best_orig_pos, best_dest_pos);
            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }

    }

    return general_improvement;
}

double LocalSearches::UN_1opt_delta_cost(Solution & solution, const int vhc, const int in_pos, const int out_pos) {
    double unload_time;
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);

    Schedules & schedules = solution.schedule_manager_.schedules_;

    if (in_pos < out_pos) {
        req_start_unload[schedules[vhc].unloaded_reqs_[out_pos].ind_] = schedules[vhc].unloaded_reqs_[in_pos].start_time_;
        unload_time = Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[out_pos].ind_);

        for (int i = in_pos; i < out_pos; ++i) {
            req_start_unload[schedules[vhc].unloaded_reqs_[i].ind_] = schedules[vhc].unloaded_reqs_[i].start_time_ + unload_time;
        }
    } else {
        unload_time = Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[out_pos].ind_);

        for (int i = out_pos + 1; i < in_pos; ++i) {
            req_start_unload[schedules[vhc].unloaded_reqs_[i].ind_] = schedules[vhc].unloaded_reqs_[i].start_time_ - unload_time;
        }

        req_start_unload[schedules[vhc].unloaded_reqs_[out_pos].ind_] = req_start_unload[schedules[vhc].unloaded_reqs_[in_pos - 1].ind_]
                                                                        + Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[in_pos - 1].ind_);
    }

    set<int> impacted_reloads;
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double delta_cost = 0.0;
    double curr_delta, infeas_st_av_size;

    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
        ReloadSchedule rs = schedules[*it].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*it, vhc);
            }
        }

        sort(rs.begin(), rs.end());
        rs[0].start_time_ = schedules[*it].unload_end_ + Instance::instance()->reload_preparation_time();
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

        curr_delta = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*it].completion_time_;
        delta_cost += curr_delta;
    }

    return delta_cost;
}

void LocalSearches::UN_move_1opt(Solution & solution, const int k, const int selected_pos, int best_pos) {
    double unload_time;
    Schedules & schedules = solution.schedule_manager_.schedules_;
    UnloadUnit selected_req = schedules[k].unloaded_reqs_[selected_pos];
    set<int> impacted_reloads;

    if (best_pos > selected_pos) {
        --best_pos;
    }
    schedules[k].unloaded_reqs_.erase(schedules[k].unloaded_reqs_.begin() + selected_pos);
    schedules[k].unloaded_reqs_.insert(schedules[k].unloaded_reqs_.begin() + best_pos, selected_req);

    int dest_vhc;
    unload_time = Instance::instance()->unloading_time(selected_req.ind_);
    if (best_pos < selected_pos) {
        for (int i = best_pos + 1; i <= selected_pos; ++i) {
            schedules[k].unloaded_reqs_[i].start_time_ += unload_time;
            dest_vhc = solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[i].ind_].second;

            for (uint j = 0; j < schedules[dest_vhc].reloaded_reqs_.size(); ++j) {
                if (schedules[k].unloaded_reqs_[i].ind_ == schedules[dest_vhc].reloaded_reqs_[j].ind_) {
                    schedules[dest_vhc].reloaded_reqs_[j].availability_time_ += unload_time;
                    break;
                }
            }
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[i].ind_].second);
        }
        schedules[k].unloaded_reqs_[best_pos].start_time_ = schedules[k].unloaded_reqs_[best_pos + 1].start_time_ - unload_time;
        dest_vhc = solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[best_pos].ind_].second;
        for (uint j = 0; j < schedules[dest_vhc].reloaded_reqs_.size(); ++j) {
            if (schedules[k].unloaded_reqs_[best_pos].ind_ == schedules[dest_vhc].reloaded_reqs_[j].ind_) {
                schedules[dest_vhc].reloaded_reqs_[j].availability_time_ = schedules[k].unloaded_reqs_[best_pos].start_time_
                                                                           + Instance::instance()->unloading_time(schedules[k].unloaded_reqs_[best_pos].ind_)
                                                                           + Instance::instance()->between_docks_time(k, dest_vhc);
                break;
            }
        }
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[best_pos].ind_].second);
    } else {
        for (int i = selected_pos; i < best_pos; ++i) {
            schedules[k].unloaded_reqs_[i].start_time_ -= unload_time;
            dest_vhc = solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[i].ind_].second;

            for (uint j = 0; j < schedules[dest_vhc].reloaded_reqs_.size(); ++j) {
                if (schedules[k].unloaded_reqs_[i].ind_ == schedules[dest_vhc].reloaded_reqs_[j].ind_) {
                    schedules[dest_vhc].reloaded_reqs_[j].availability_time_ -= unload_time;
                    break;
                }
            }
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[i].ind_].second);
        }
        schedules[k].unloaded_reqs_[best_pos].start_time_ = schedules[k].unloaded_reqs_[best_pos - 1].start_time_ 
                                                            + Instance::instance()->unloading_time(schedules[k].unloaded_reqs_[best_pos - 1].ind_);
        dest_vhc = solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[best_pos].ind_].second;
        for (uint j = 0; j < schedules[dest_vhc].reloaded_reqs_.size(); ++j) {
            if (schedules[k].unloaded_reqs_[best_pos].ind_ == schedules[dest_vhc].reloaded_reqs_[j].ind_) {
                schedules[dest_vhc].reloaded_reqs_[j].availability_time_ = schedules[k].unloaded_reqs_[best_pos].start_time_
                                                                           + Instance::instance()->unloading_time(schedules[k].unloaded_reqs_[best_pos].ind_)
                                                                           + Instance::instance()->between_docks_time(k, dest_vhc);
                break;
            }
        }
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[best_pos].ind_].second);
    }

    double curr_delta = 0.0, total_delta = 0.0, infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

        curr_delta = (schedules[*it].reloaded_reqs_[schedules[*it].reloaded_reqs_.size() - 1].start_time_
                      + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_[schedules[*it].reloaded_reqs_.size() - 1].ind_))
                     - schedules[*it].completion_time_;
        schedules[*it].completion_time_ += curr_delta;
        total_delta += curr_delta;
    }

    solution.cost_ += total_delta;
}

bool LocalSearches::UN_Exchange(Solution & solution, LS_strat search_strat) {

    int num_requests = Instance::instance()->num_requests();
    int num_vehicles = Instance::instance()->num_vehicles();
    bool improve, general_improvement;
    int num_unloaded_reqs, curr_pos = -1, curr_vhc, best_vhc, best_pos_1, best_pos_2;
    double best_cost, temp_cost, current_cost;
    vector<int> selectable_reqs;
    vector<bool> reqs_already_selected(num_requests + 1);

    for (int k = 0; k < num_vehicles; ++k) {
        for (unsigned int i = 0; i < solution.schedule_manager_.schedules_[k].unloaded_reqs_.size(); ++i) {
            selectable_reqs.push_back(solution.schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_);
        }
    }
    shuffle(selectable_reqs.begin(), selectable_reqs.end(), rand_utils::generator);

    general_improvement = false;
    improve = true;
    while (improve) {
        improve = false;
        best_cost = solution.cost_;
        reqs_already_selected.assign(num_requests + 1, false);

        for (auto selected_req = selectable_reqs.begin(); selected_req != selectable_reqs.end(); ++selected_req) {
            reqs_already_selected[*selected_req] = true;
            curr_vhc = solution.schedule_manager_.pic_del_[*selected_req].first;
            num_unloaded_reqs = solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_.size();
            for (int i = 0; i < num_unloaded_reqs; ++i) {
                if (solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_[i].ind_ == *selected_req) {
                    curr_pos = i;
                    break;
                }
            }

            temp_cost = solution.cost_;

            for (int i = 0; i < num_unloaded_reqs; ++i) {
                if (!reqs_already_selected[solution.schedule_manager_.schedules_[curr_vhc].unloaded_reqs_[i].ind_]) {
                    current_cost = temp_cost + UN_exchange_delta_cost(solution, curr_vhc, curr_pos, i);

                    if (cmp(current_cost, best_cost) < 0) {
                        best_cost = current_cost;
                        best_pos_1 = curr_pos;
                        best_pos_2 = i;
                        best_vhc = curr_vhc;
                        improve = true;
                    }
                }
            }

            if (improve && (search_strat == LS_strat::complete_first_improv_ || search_strat == LS_strat::one_first_improv_)) {
                general_improvement = true;
                UN_move_exchange(solution, best_vhc, best_pos_1, best_pos_2);
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
            UN_move_exchange(solution, best_vhc, best_pos_1, best_pos_2);

            if (search_strat == LS_strat::one_best_improv_) {
                return general_improvement;
            }
        }

    }

    return general_improvement;
}

double LocalSearches::UN_exchange_delta_cost(Solution & solution, const int vhc, const int orig_pos, const int dest_pos) {
    double diff_time;
    int num_reqs = Instance::instance()->num_requests();
    vector<double> req_start_unload(num_reqs + 1, -1.0);
    int begin, end;
    Schedules & schedules = solution.schedule_manager_.schedules_;

    if (orig_pos < dest_pos) {
        begin = orig_pos;
        end = dest_pos;
    } else {
        begin = dest_pos;
        end = orig_pos;
    }

    req_start_unload[schedules[vhc].unloaded_reqs_[end].ind_] = schedules[vhc].unloaded_reqs_[begin].start_time_;

    diff_time = Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[end].ind_)
                - Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[begin].ind_);

    for (int i = begin + 1; i < end; ++i) {
        req_start_unload[schedules[vhc].unloaded_reqs_[i].ind_] = schedules[vhc].unloaded_reqs_[i].start_time_ + diff_time;
    }

    if (end - begin > 1) {
        req_start_unload[schedules[vhc].unloaded_reqs_[begin].ind_] = req_start_unload[schedules[vhc].unloaded_reqs_[end - 1].ind_]
                                                                      + Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[end - 1].ind_);
    } else {
        req_start_unload[schedules[vhc].unloaded_reqs_[begin].ind_] = schedules[vhc].unloaded_reqs_[begin].start_time_
                                                                      + Instance::instance()->unloading_time(schedules[vhc].unloaded_reqs_[end].ind_);
    }

    set<int> impacted_reloads;
    for (int i = 1; i <= num_reqs; ++i) {
        if (req_start_unload[i] >= 0.0) {
            impacted_reloads.insert(solution.schedule_manager_.pic_del_[i].second);
        }
    }

    double delta_cost = 0.0;
    double curr_delta, infeas_st_av_size;

    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
        ReloadSchedule rs = schedules[*it].reloaded_reqs_;
        for (uint i = 0; i < rs.size(); ++i) {
            if (req_start_unload[rs[i].ind_] >= 0.0) {
                rs[i].availability_time_ = req_start_unload[rs[i].ind_] + Instance::instance()->unloading_time(rs[i].ind_) + Instance::instance()->between_docks_time(*it, vhc);
            }
        }

        sort(rs.begin(), rs.end());
        rs[0].start_time_ = schedules[*it].unload_end_ + Instance::instance()->reload_preparation_time();
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

        curr_delta = (rs[rs.size() - 1].start_time_ + Instance::instance()->reloading_time(rs[rs.size() - 1].ind_)) - schedules[*it].completion_time_;
        delta_cost += curr_delta;
    }

    return delta_cost;
}

void LocalSearches::UN_move_exchange(Solution & solution, const int k, const int orig_pos, const int dest_pos) {
    set<int> impacted_reloads;
    int begin, end, dest_vhc;
    Schedules & schedules = solution.schedule_manager_.schedules_;

    if (orig_pos < dest_pos) {
        begin = orig_pos;
        end = dest_pos;
    } else {
        begin = dest_pos;
        end = orig_pos;
    }

    schedules[k].unloaded_reqs_[end].start_time_ = schedules[k].unloaded_reqs_[begin].start_time_;

    swap(schedules[k].unloaded_reqs_[orig_pos], schedules[k].unloaded_reqs_[dest_pos]);

    dest_vhc = solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[begin].ind_].second;

    for (uint j = 0; j < schedules[dest_vhc].reloaded_reqs_.size(); ++j) {
        if (schedules[k].unloaded_reqs_[begin].ind_ == schedules[dest_vhc].reloaded_reqs_[j].ind_) {
            schedules[dest_vhc].reloaded_reqs_[j].availability_time_ = schedules[k].unloaded_reqs_[begin].start_time_
                                                                       + Instance::instance()->unloading_time(schedules[k].unloaded_reqs_[begin].ind_)
                                                                       + Instance::instance()->between_docks_time(k, dest_vhc);;
            break;
        }
    }
    impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[begin].ind_].second);

    for (int i = begin + 1; i <= end; ++i) {
        schedules[k].unloaded_reqs_[i].start_time_ = schedules[k].unloaded_reqs_[i-1].start_time_
                                                     + Instance::instance()->unloading_time(schedules[k].unloaded_reqs_[i-1].ind_);

        dest_vhc = solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[i].ind_].second;

        for (uint j = 0; j < schedules[dest_vhc].reloaded_reqs_.size(); ++j) {
            if (schedules[k].unloaded_reqs_[i].ind_ == schedules[dest_vhc].reloaded_reqs_[j].ind_) {
                schedules[dest_vhc].reloaded_reqs_[j].availability_time_ = schedules[k].unloaded_reqs_[i].start_time_
                                                                           + Instance::instance()->unloading_time(schedules[k].unloaded_reqs_[i].ind_)
                                                                           + Instance::instance()->between_docks_time(k, dest_vhc);;
                break;
            }
        }
        impacted_reloads.insert(solution.schedule_manager_.pic_del_[schedules[k].unloaded_reqs_[i].ind_].second);
    }

    double curr_delta = 0.0, total_delta = 0.0, infeas_st_av_size;
    for (auto it = impacted_reloads.begin(); it != impacted_reloads.end(); ++it) {
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

        curr_delta = (schedules[*it].reloaded_reqs_[schedules[*it].reloaded_reqs_.size() - 1].start_time_
                      + Instance::instance()->reloading_time(schedules[*it].reloaded_reqs_[schedules[*it].reloaded_reqs_.size() - 1].ind_))
                     - schedules[*it].completion_time_;
        schedules[*it].completion_time_ += curr_delta;
        total_delta += curr_delta;
    }

    solution.cost_ += total_delta;
}
