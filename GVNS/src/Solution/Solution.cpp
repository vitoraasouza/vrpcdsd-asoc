#include "Solution.h"

Solution::Solution(const int num_requests, const int num_vehicles) {
    routes_pickup_.resize(num_vehicles);
    routes_delivery_.resize(num_vehicles);
    schedule_manager_ = ScheduleManager(num_requests, num_vehicles);
    num_vehicles_ = num_vehicles;
    cost_ = 0.0;
}

void Solution::printTikZSolutionGuide(const char * tikz_guide_file_name) {
    FILE * tikz_file = fopen(tikz_guide_file_name, "w");
    int num_requests = Instance::instance()->num_requests();

    vector<bool> do_unload(num_vehicles_, false), do_reload(num_vehicles_, false);
    for(int k = 0; k < num_vehicles_; ++k) {
        if (schedule_manager_.schedules_[k].unloaded_reqs_.size() > 0) {
            do_unload[k] = true;
        }
    }
    for(int k = 0; k < num_vehicles_; ++k) {
        if (schedule_manager_.schedules_[k].reloaded_reqs_.size() > 0) {
            do_reload[k] = true;
        }
    }

    fprintf(tikz_file, "INSTANCE_NAME: %s\n", Instance::instance()->name());
    fprintf(tikz_file, "NUM_REQUESTS: %d\n", num_requests);
    fprintf(tikz_file, "NUM_VEHICLES: %d\n", num_vehicles_);
    fprintf(tikz_file, "SOLUTION_COST: %.3lf\n", cost_);
    fprintf(tikz_file, "CRITERION: total completion time\n\n");


    fprintf(tikz_file, "COORDINATES(SUPPLIER-CUSTOMER)\n");
    fprintf(tikz_file, "%3d: %10.1lf %10.1lf\n", 0, Instance::instance()->node(0).x(), Instance::instance()->node(0).y());
    for (int i = 1; i <= num_requests; ++i) {
        fprintf(tikz_file, "%3d: %10.1lf %10.1lf %10.1lf %10.1lf %10d\n", i, Instance::instance()->node(i).x(), Instance::instance()->node(i).y(),
                Instance::instance()->node(i+num_requests).x(), Instance::instance()->node(i+num_requests).y(), Instance::instance()->node(i).demand());
    }

    fprintf(tikz_file, "\nROUTES_PICKUP\n");

    for (int k = 0; k < num_vehicles_; ++k) {
        fprintf(tikz_file, "k%d:    C: %8.3lf    L: %3d    ", k+1, routes_pickup_[k].cost_, routes_pickup_[k].load_);
        fprintf(tikz_file, "(0 ");

        for (int i = Route::first_insertable_pos_; i <= routes_pickup_[k].num_nodes_; ++i) {
            fprintf(tikz_file, "%d ", routes_pickup_[k].visited_nodes_[i]);
        }
        fprintf(tikz_file, "0)\n");
    }

    fprintf(tikz_file, "\nROUTES_DELIVERY\n");

    for (int k = 0; k < num_vehicles_; ++k) {
        fprintf(tikz_file, "k%d:    C: %8.3lf    L: %3d    ", k+1, routes_delivery_[k].cost_, routes_delivery_[k].load_);
        fprintf(tikz_file, "(0 ");

        for (int i = Route::first_insertable_pos_; i <= routes_delivery_[k].num_nodes_; ++i) {
            fprintf(tikz_file, "%d ", routes_delivery_[k].visited_nodes_[i] - num_requests);
        }
        fprintf(tikz_file, "0)\n");
    }

    fprintf(tikz_file, "\nSCHEDULING\n");

    double start_scheduling = 1000000000.0;
    double makespan = 0.0;
    for (int k = 0; k < num_vehicles_; ++k) {
        if (cmp(schedule_manager_.schedules_[k].start_time_, start_scheduling) < 0 && (do_unload[k] || do_reload[k])) {
            start_scheduling = schedule_manager_.schedules_[k].start_time_;
        }
        if (cmp(makespan, schedule_manager_.schedules_[k].completion_time_) < 0 && (do_unload[k] || do_reload[k])) {
            makespan = schedule_manager_.schedules_[k].completion_time_;
        }
    }

    fprintf(tikz_file, "START_SCHEDULING: %.3lf\n", (start_scheduling == 1000000000.0 ? -1: start_scheduling));
    fprintf(tikz_file, "MAKESPAN: %.3lf\n\n", makespan);

    fprintf(tikz_file, "PROCESSING_TIMES\n");
    for (int k = 0; k < num_vehicles_; ++k) {
        if (cmp(schedule_manager_.schedules_[k].start_time_ - start_scheduling, 0.0) > 0 && (do_unload[k] || do_reload[k])) {
            fprintf(tikz_file, "T %5d %14.3lf %8.3lf\n", k+1, start_scheduling, schedule_manager_.schedules_[k].start_time_ - start_scheduling);
        }
    }

    for (int k = 0; k < num_vehicles_; ++k) {
        if (do_unload[k]) {
            for (uint i = 0; i < schedule_manager_.schedules_[k].unloaded_reqs_.size(); ++i) {
                fprintf(tikz_file, "U %5d %5d %8.3lf %8d\n", k+1, schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_, schedule_manager_.schedules_[k].unloaded_reqs_[i].start_time_, Instance::instance()->unloading_time(schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_));
            }
        }
    }

    for (int k = 0; k < num_vehicles_; ++k) {
        if (do_reload[k]) {
            for (uint i = 0; i < schedule_manager_.schedules_[k].reloaded_reqs_.size(); ++i) {
                fprintf(tikz_file, "R %5d %5d %8.3lf %8d\n", k+1, schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_, schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_, Instance::instance()->reloading_time(schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_));
            }
        }
    }

    for (int k = 0; k < num_vehicles_; ++k) {
        if (do_unload[k]) {
            fprintf(tikz_file, "P %5d %14.3lf %8.3lf\n", k+1, schedule_manager_.schedules_[k].start_time_, (double)Instance::instance()->unload_preparation_time());
        }
    }

    for (int k = 0; k < num_vehicles_; ++k) {
        if (do_reload[k]) {
            fprintf(tikz_file, "P %5d %14.3lf %8.3lf\n", k+1, schedule_manager_.schedules_[k].unload_end_, (double)Instance::instance()->reload_preparation_time());
        }
    }

    fclose(tikz_file);
}

bool Solution::checkFeasibility() {
    bool is_feasible = true;
    int num_vehicles = Instance::instance()->num_vehicles();
    int num_requests = Instance::instance()->num_requests();
    int vhc_capacity = Instance::instance()->vehicle().capacity();
    vector<bool> visited_sup(num_requests + 1, false), visited_cus(num_requests + 1, false);
    int num_dif_sup = 0, num_dif_cus = 0;
    double pic_cost = 0.0, del_cost = 0.0, sched_cost = 0.0, total_cost;
    vector<double> pic_routes_costs(num_vehicles, 0.0), del_routes_costs(num_vehicles, 0.0);
    vector<double> start_time(num_vehicles, 0.0), unload_end(num_vehicles, 0.0), comp_time(num_vehicles, 0.0);
    vector<pair<int, int>> pic_del(num_requests + 1);
    vector<double> start_unl(num_requests + 1, 0.0), start_rel(num_requests + 1, 0.0), avail_time(num_requests + 1, 0.0);

    if (static_cast<int>(routes_pickup_.size()) != num_vehicles) {
        cout << "Numero de veiculos de coleta na solucao = " << routes_pickup_.size() << " | valor correto = " << num_vehicles << endl;
        is_feasible = false;
    }

    if (static_cast<int>(routes_delivery_.size()) != num_vehicles) {
        cout << "Numero de veiculos de entrega na solucao = " << routes_delivery_.size() << " | valor correto = " << num_vehicles << endl;
        is_feasible = false;
    }

    for (int k = 0; k < num_vehicles; ++k) {
        if (static_cast<int>(routes_pickup_[k].visited_nodes_.size()) - 2 != routes_pickup_[k].num_nodes_) {
            cout << "num_nodes (" << routes_pickup_[k].num_nodes_ << ") difere do numero de visited_nodes (" << routes_pickup_[k].visited_nodes_.size() - 2 << " - sem o CD) no veiculo " << k+1 << " de coleta" << endl;
            is_feasible = false;
        }
        if (static_cast<int>(routes_pickup_[k].visited_nodes_.size()) - 1 != routes_pickup_[k].last_insertable_pos_) {
            cout << "last_insertable_pos (" << routes_pickup_[k].last_insertable_pos_ << ") difere do valor que deveria ser (" << routes_pickup_[k].visited_nodes_.size() - 1 << ")" << endl;
            is_feasible = false;
        }
        if (routes_pickup_[k].visited_nodes_[0] != 0) {
            cout << "O primeiro vertice visitado pelo veiculo " << k+1 << " da coleta nao e o CD" << endl;
            is_feasible = false;
        }
        if (routes_pickup_[k].visited_nodes_[routes_pickup_[k].visited_nodes_.size() - 1] != 0) {
            cout << "O ultimo vertice visitado pelo veiculo " << k+1 << " da coleta nao e o CD" << endl;
            is_feasible = false;
        }
        for (int i = 1; i < static_cast<int>(routes_pickup_[k].visited_nodes_.size()) - 1; ++i) {
            if (!visited_sup[routes_pickup_[k].visited_nodes_[i]]) {
                visited_sup[routes_pickup_[k].visited_nodes_[i]] = true;
                ++num_dif_sup;
            } else {
                cout << "O fornecedor " << routes_pickup_[k].visited_nodes_[i] << " foi visitado mais de uma vez" << endl;
                is_feasible = false;
            }
        }
    }

    if (num_dif_sup != num_requests) {
        for (int r = 1; r <= num_requests; ++r) {
            if (!visited_sup[r]) {
                cout << "O fornecedor " << r << " nao foi visitado" << endl;
                is_feasible = false;
            }
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        if (static_cast<int>(routes_delivery_[k].visited_nodes_.size()) - 2 != routes_delivery_[k].num_nodes_) {
            cout << "num_nodes (" << routes_delivery_[k].num_nodes_ << ") difere do numero de visited_nodes (" << routes_delivery_[k].visited_nodes_.size() - 2 << " - sem o CD) no veiculo " << k+1 << " de entrega" << endl;
            is_feasible = false;
        }
        if (static_cast<int>(routes_delivery_[k].visited_nodes_.size()) - 1 != routes_delivery_[k].last_insertable_pos_) {
            cout << "last_insertable_pos (" << routes_delivery_[k].last_insertable_pos_ << ") difere do valor que deveria ser (" << routes_delivery_[k].visited_nodes_.size() - 1 << ")" << endl;
            is_feasible = false;
        }
        if (routes_delivery_[k].visited_nodes_[0] != 0) {
            cout << "O primeiro vertice visitado pelo veiculo " << k+1 << " da entrega nao e o CD" << endl;
            is_feasible = false;
        }
        if (routes_delivery_[k].visited_nodes_[routes_delivery_[k].visited_nodes_.size() - 1] != 0) {
            cout << "O ultimo vertice visitado pelo veiculo " << k+1 << " da entrega nao e o CD" << endl;
            is_feasible = false;
        }
        for (int i = 1; i < static_cast<int>(routes_delivery_[k].visited_nodes_.size()) - 1; ++i) {
            if (!visited_cus[routes_delivery_[k].visited_nodes_[i] - num_requests]) {
                visited_cus[routes_delivery_[k].visited_nodes_[i] - num_requests] = true;
                ++num_dif_cus;
            } else {
                cout << "O cliente " << routes_delivery_[k].visited_nodes_[i] << " foi visitado mais de uma vez" << endl;
                is_feasible = false;
            }
        }
    }

    if (num_dif_cus != num_requests) {
        for (int r = 1; r <= num_requests; ++r) {
            if (!visited_cus[r]) {
                cout << "O cliente " << r << " nao foi visitado" << endl;
                is_feasible = false;
            }
        }
    }

    int load;
    for (int k = 0; k < num_vehicles; ++k) {
        load = 0;
        for (int i = 0; i < static_cast<int>(routes_pickup_[k].visited_nodes_.size()) - 1; ++i) {
            load += Instance::instance()->node(routes_pickup_[k].visited_nodes_[i]).demand();
            pic_routes_costs[k] += Instance::instance()->EdgeCost(routes_pickup_[k].visited_nodes_[i], routes_pickup_[k].visited_nodes_[i+1]);
        }
        if (load != routes_pickup_[k].load_) {
            cout << "A carga avaliada (" << load << ") do veiculo " << k+1 << " da coleta eh diferente da carga armazenada na solucao (" << routes_pickup_[k].load_ << ")" << endl;
            is_feasible = false;
        }
        if (load > vhc_capacity) {
            cout << "A carga (" << load << ") do veiculo " << k+1 << " de coleta excede a sua capacidade (" << vhc_capacity << ")" << endl;
            is_feasible = false;
        }
        if (cmp(pic_routes_costs[k], routes_pickup_[k].cost_) != 0) {
            cout << "O custo avaliado (" << pic_routes_costs[k] << ") do veiculo " << k+1 << " da coleta eh diferente do custo armazenado na solucao (" << routes_pickup_[k].cost_ << ")" << endl;
            is_feasible = false;
        }
        pic_cost += pic_routes_costs[k];
    }

    for (int k = 0; k < num_vehicles; ++k) {
        load = 0;
        for (int i = 0; i < static_cast<int>(routes_delivery_[k].visited_nodes_.size()) - 1; ++i) {
            load += Instance::instance()->node(routes_delivery_[k].visited_nodes_[i]).demand();
            del_routes_costs[k] += Instance::instance()->EdgeCost(routes_delivery_[k].visited_nodes_[i], routes_delivery_[k].visited_nodes_[i+1]);
        }
        if (load != routes_delivery_[k].load_) {
            cout << "A carga avaliada (" << load << ") do veiculo " << k+1 << " da entrega eh diferente da carga armazenada na solucao (" << routes_delivery_[k].load_ << ")" << endl;
            is_feasible = false;
        }
        if (load > vhc_capacity) {
            cout << "A carga (" << load << ") do veiculo " << k+1 << " de entrega excede a sua capacidade (" << vhc_capacity << ")" << endl;
            is_feasible = false;
        }
        if (cmp(del_routes_costs[k], routes_delivery_[k].cost_) != 0) {
            cout << "O custo avaliado (" << del_routes_costs[k] << ") do veiculo " << k+1 << " da entrega eh diferente do custo armazenado na solucao (" << routes_delivery_[k].cost_ << ")" << endl;
            is_feasible = false;
        }
        del_cost += del_routes_costs[k];
    }

    for (int k = 0; k < num_vehicles; ++k) {
        start_time[k] = pic_routes_costs[k];
        if (cmp(start_time[k], schedule_manager_.schedules_[k].start_time_) != 0) {
            cout << "O start_time (" << start_time[k] << ") avaliado da porta " << k+1 << " esta diferente do armazenado na solucao (" << schedule_manager_.schedules_[k].start_time_ << ")" << endl;
            is_feasible = false;
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 1; i < static_cast<int>(routes_pickup_[k].visited_nodes_.size()) - 1; ++i) {
            pic_del[routes_pickup_[k].visited_nodes_[i]].first = k;
        }
        for (int i = 1; i < static_cast<int>(routes_delivery_[k].visited_nodes_.size()) - 1; ++i) {
            pic_del[routes_delivery_[k].visited_nodes_[i] - num_requests].second = k;
        }
    }

    for (int r = 1; r <= num_requests; ++r) {
        if (pic_del[r].first != schedule_manager_.pic_del_[r].first) {
            cout << "O pic_del[" << r << "].first avaliado (" << pic_del[r].first << ") eh diferente do armazenado na solucao (" << schedule_manager_.pic_del_[r].first << ")" << endl;
            is_feasible = false;
        }
        if (pic_del[r].second != schedule_manager_.pic_del_[r].second) {
            cout << "O pic_del[" << r << "].second avaliado (" << pic_del[r].second << ") eh diferente do armazenado na solucao (" << schedule_manager_.pic_del_[r].second << ")" << endl;
            is_feasible = false;
        }
    }

    int pic_vhc, del_vhc;
    bool found;
    for (int r = 1; r <= num_requests; ++r) {
        if (pic_del[r].first != pic_del[r].second) {
            pic_vhc = pic_del[r].first;
            found = false;
            for (int i = 0; i < static_cast<int>(schedule_manager_.schedules_[pic_vhc].unloaded_reqs_.size()); ++i) {
                if (r == schedule_manager_.schedules_[pic_vhc].unloaded_reqs_[i].ind_) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                cout << "A requisicao " << r << " deveria estar sendo descarregada na porta " << pic_vhc+1 << endl;
                is_feasible = false;
            }

            del_vhc = pic_del[r].second;
            found = false;
            for (int i = 0; i < static_cast<int>(schedule_manager_.schedules_[del_vhc].reloaded_reqs_.size()); ++i) {
                if (r == schedule_manager_.schedules_[del_vhc].reloaded_reqs_[i].ind_) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                cout << "A requisicao " << r << " deveria estar sendo recarregada na porta " << del_vhc+1 << endl;
                is_feasible = false;
            }
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 0; i < static_cast<int>(schedule_manager_.schedules_[k].unloaded_reqs_.size()); ++i) {
            if (pic_del[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_].first != k) {
                cout << "A requisicao " << schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_ << " esta sendo descarregada erroneamente na porta " << k+1 << endl;
                is_feasible = false;
            }
        }
        for (int i = 0; i < static_cast<int>(schedule_manager_.schedules_[k].reloaded_reqs_.size()); ++i) {
            if (pic_del[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_].second != k) {
                cout << "A requisicao " << schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_ << " esta sendo recarregada erroneamente na porta " << k+1 << endl;
                is_feasible = false;
            }
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 0; i < static_cast<int>(schedule_manager_.schedules_[k].unloaded_reqs_.size()); ++i) {
            if (i == 0) {
                start_unl[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_] = start_time[k] + Instance::instance()->unload_preparation_time();
            } else {
                start_unl[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_] = start_unl[schedule_manager_.schedules_[k].unloaded_reqs_[i-1].ind_] + Instance::instance()->unloading_time(schedule_manager_.schedules_[k].unloaded_reqs_[i-1].ind_);
            }

            if (cmp(start_unl[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_], schedule_manager_.schedules_[k].unloaded_reqs_[i].start_time_) != 0) {
                cout << "O inicio do descarregamento da requisicao " << schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_ << " avaliado (" << start_unl[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_]
                     << ") esta diferente do valor armazenado na solucao (" << schedule_manager_.schedules_[k].unloaded_reqs_[i].start_time_ << ")" << endl;
                is_feasible = false;
            }

            avail_time[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_] = start_unl[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_]
                                                                                 + Instance::instance()->unloading_time(schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_)
                                                                                 + Instance::instance()->between_docks_time(k, pic_del[schedule_manager_.schedules_[k].unloaded_reqs_[i].ind_].second);
        }

        if (schedule_manager_.schedules_[k].unloaded_reqs_.size() > 0) {
            unload_end[k] = start_unl[schedule_manager_.schedules_[k].unloaded_reqs_.back().ind_] + Instance::instance()->unloading_time(schedule_manager_.schedules_[k].unloaded_reqs_.back().ind_);
        } else {
            unload_end[k] = start_time[k];
        }
        if (cmp(unload_end[k], schedule_manager_.schedules_[k].unload_end_) != 0) {
            cout << "O unload_end avaliado (" << unload_end[k] << ") da porta " << k+1 << " eh diferente do valor armazenado na solucao (" << schedule_manager_.schedules_[k].unload_end_ << ")" << endl;
            is_feasible = false;
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        for (int i = 0; i < static_cast<int>(schedule_manager_.schedules_[k].reloaded_reqs_.size()); ++i) {
            if (i == 0) {
                start_rel[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_] = max(unload_end[k] + Instance::instance()->reload_preparation_time(), avail_time[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_]);
            } else {
                start_rel[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_] = max(start_rel[schedule_manager_.schedules_[k].reloaded_reqs_[i-1].ind_]
                                                                                    + Instance::instance()->reloading_time(schedule_manager_.schedules_[k].reloaded_reqs_[i-1].ind_),
                                                                                    avail_time[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_]);
            }

            if (cmp(avail_time[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_], schedule_manager_.schedules_[k].reloaded_reqs_[i].availability_time_) != 0) {
                cout << "A disponibilidade de recarregamento da requisicao " << schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_ << " avaliada (" << avail_time[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_]
                     << ") esta diferente do valor armazenado na solucao (" << schedule_manager_.schedules_[k].reloaded_reqs_[i].availability_time_ << ")" << endl;
                is_feasible = false;
            }
        }
    }

    for (int k = 0; k < num_vehicles; ++k) {
        if (schedule_manager_.schedules_[k].reloaded_reqs_.size() > 0) {
            double curr_reload_st = start_rel[schedule_manager_.schedules_[k].reloaded_reqs_.back().ind_];
            for (int i = static_cast<int>(schedule_manager_.schedules_[k].reloaded_reqs_.size() - 1); i >= 0; --i) {
                if (i == static_cast<int>(schedule_manager_.schedules_[k].reloaded_reqs_.size() - 1)) {
                    if (cmp(start_rel[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_], schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_) != 0) {
                        cout << "O inicio do recarregamento da requisicao " << schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_ << " avaliado (" << start_rel[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_]
                             << ") esta diferente do valor armazenado na solucao (" << schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_ << ")" << endl;
                        is_feasible = false;
                    }
                } else {
                    curr_reload_st -= Instance::instance()->reloading_time(schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_);
                    if (cmp(schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_, start_rel[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_]) < 0
                        || cmp(schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_, curr_reload_st) > 0) {

                        cout << "O inicio do recarregamento da requisicao " << schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_ << " avaliado no intervalo [" << start_rel[schedule_manager_.schedules_[k].reloaded_reqs_[i].ind_]
                             << "," << curr_reload_st << "] esta diferente do valor armazenado na solucao (" << schedule_manager_.schedules_[k].reloaded_reqs_[i].start_time_ << ")" << endl;
                        is_feasible = false;
                    }
                }
            }

            comp_time[k] = start_rel[schedule_manager_.schedules_[k].reloaded_reqs_.back().ind_] + Instance::instance()->reloading_time(schedule_manager_.schedules_[k].reloaded_reqs_.back().ind_);
        } else {
            comp_time[k] = unload_end[k];
        }
        if (cmp(comp_time[k], schedule_manager_.schedules_[k].completion_time_) != 0) {
            cout << "O completion_time avaliado (" << comp_time[k] << ") da porta " << k+1 << " eh diferente do valor armazenado na solucao (" << schedule_manager_.schedules_[k].completion_time_ << ")" << endl;
            is_feasible = false;
        }

        sched_cost += comp_time[k];
    }

    total_cost = del_cost + sched_cost;

    if (cmp(total_cost, cost_) != 0) {
        cout << "A FO avaliada (" << total_cost << ") eh diferente da armazenada na solucao (" << cost_ << ")" << endl;
        is_feasible = false;
    }

    return is_feasible;
}

Solution & Solution::operator=(const Solution & solution) {
    routes_pickup_ = solution.routes_pickup_;
    routes_delivery_ = solution.routes_delivery_;
    schedule_manager_ = solution.schedule_manager_;
    num_vehicles_ = solution.num_vehicles_;
    cost_ = solution.cost_;

    return * this;
}

ostream & operator<<(ostream & out, const Solution & solution) {
    out << ">>> PICKUP ROUTES:" << endl;
    out << solution.routes_pickup_ << endl;
    out << ">>> DELIVERY ROUTES" << endl;
    out << solution.routes_delivery_ << endl;
    out << ">>> SCHEDULE" << endl;
    out << solution.schedule_manager_ << endl;
    out << "Obj. function = " << solution.cost_ << endl;

    return out;
}
