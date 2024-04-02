#include "CompactModel.h"

CompactModel::CompactModel(InputSettings input_settings) :
    master_(env_) {

    try {
        master_cpx_ = new IloCplex(master_);

        input_settings_ = input_settings;

        num_vehicles_ = Instance::instance()->num_vehicles();
        num_real_nodes_ = Instance::instance()->num_nodes();
        num_model_nodes_ = num_real_nodes_ + 1; // node 0 is duplicated
        num_suppliers_ = Instance::instance()->num_suppliers();
        num_customers_ = Instance::instance()->num_customers();
        num_requests_ = Instance::instance()->num_requests();

        num_edges_ = 2 * (((num_requests_ + 2) * (num_requests_ + 1)) / 2 - 1);

        first_supplier_ = 1;
        last_supplier_ = num_suppliers_;

        first_customer_ = last_supplier_ + 1;
        last_customer_ = last_supplier_ + num_customers_;

        orig_ = 0;
        dest_ = 2 * num_requests_ + 1;

        report_infos_.num_threads_ = input_settings.num_threads_;

        int curr_ind = 0;
        x_index_.resize(num_model_nodes_);
        for (int i = 0; i < num_model_nodes_; ++i) {
            x_index_[i].resize(num_model_nodes_, -1);
        }

        for (int i = orig_; i <= last_supplier_; ++i) {
            for (int j = i + 1; j <= last_supplier_; ++j) {
                x_index_[i][j] = x_index_[j][i] = curr_ind++;
            }
            if (i != orig_) {
                x_index_[i][dest_] = x_index_[dest_][i] = curr_ind++;
            }
        }
        for (int i = first_customer_; i <= dest_; ++i) {
            if (i != dest_) {
                x_index_[i][orig_] = x_index_[orig_][i] = curr_ind++;
            }
            for (int j = i + 1; j <= dest_; ++j) {
                x_index_[i][j] = x_index_[j][i] = curr_ind++;
            }
        }

        curr_ind = 0;
        z_index_.resize(num_requests_ + 1);
        for (int i = 0; i <= num_requests_; ++i) {
            z_index_[i].resize(num_requests_ + 1, -1);
        }
        for (int i = 1; i <= num_requests_; ++i) {
            for (int j = i + 1; j <= num_requests_; ++j) {
                z_index_[i][j] = curr_ind++;
            }
        }

        if (input_settings.parameters_on_ == false) {
            turn_off_parameters();
        }

    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::create_model() {

    create_variables();
    create_objective_function();
    calculate_big_Ms();
    create_constraints();

}

void CompactModel::create_variables() {

    try{
        char buffer[30];

        lambda_ = IloArray<IloIntVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            lambda_[k] = IloIntVarArray(env_, num_edges_, 0 , 1);

            for (int i = 0; i <= num_suppliers_; ++i) {
                for (int j = i + 1; j <= num_suppliers_; ++j) {
                    sprintf(buffer, "x_%02d,%04d,%04d", k+1, i, j);
                    lambda_[k][x_index_[i][j]].setName(buffer);
                }
                if (i != 0) {
                    sprintf(buffer, "x_%02d,%04d,%04d", k+1, i, dest_);
                    lambda_[k][x_index_[i][dest_]].setName(buffer);
                }
            }
            for (int i = first_customer_; i <= dest_; ++i) {
                if (i != dest_) {
                    sprintf(buffer, "x_%02d,%04d,%04d", k+1, 0, i);
                    lambda_[k][x_index_[0][i]].setName(buffer);
                }
                for (int j = i + 1; j <= dest_; ++j) {
                    sprintf(buffer, "x_%02d,%04d,%04d", k+1, i, j);
                    lambda_[k][x_index_[i][j]].setName(buffer);
                }
            }
        }

        y_ = IloArray<IloIntVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            y_[k] = IloIntVarArray(env_, num_real_nodes_, 0, 1);
            for (int i = 0; i < num_real_nodes_; ++i) {
                sprintf(buffer, "y_%02d,%04d", k+1, i);

                y_[k][i].setName(buffer);
            }
        }

        f_ = IloArray<IloArray<IloNumVarArray> >(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            f_[k] = IloArray<IloNumVarArray>(env_, num_model_nodes_);
            for (int i = 0; i < num_model_nodes_; ++i) {
                f_[k][i] = IloNumVarArray(env_, num_model_nodes_, 0, IloInfinity);
                for (int j = 0; j < num_model_nodes_; ++j) {
                    sprintf(buffer, "f_%02d,%04d,%04d", k+1, i, j);

                    f_[k][i][j].setName(buffer);
                }
            }
        }

        s_ = IloArray<IloNumVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            s_[k] = IloNumVarArray(env_, num_requests_ + 1, 0, IloInfinity);
            for (int i = 0; i <= num_requests_; ++i) {
                sprintf(buffer, "su_%02d,%04d", k+1, i);

                s_[k][i].setName(buffer);
            }
        }

        sr_ = IloArray<IloNumVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            sr_[k] = IloNumVarArray(env_, num_requests_ + 1, 0, IloInfinity);
            for (int i = 0; i <= num_requests_; ++i) {
                sprintf(buffer, "sr_%02d,%04d", k+1, i);

                sr_[k][i].setName(buffer);
            }
        }

        z_ = IloArray<IloIntVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            z_[k] = IloIntVarArray(env_, (num_requests_ * (num_requests_ - 1)) / 2, 0, 1);
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j) {
                    sprintf(buffer, "zu_%02d,%04d,%04d", k+1, i, j);

                    z_[k][z_index_[i][j]].setName(buffer);
                }
            }
        }

        zr_ = IloArray<IloIntVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            zr_[k] = IloIntVarArray(env_, (num_requests_ * (num_requests_ - 1)) / 2, 0, 1);
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j) {
                    sprintf(buffer, "zr_%02d,%04d,%04d", k+1, i, j);

                    zr_[k][z_index_[i][j]].setName(buffer);
                }
            }
        }

        completion_ = IloNumVarArray(env_, num_vehicles_, 0, IloInfinity);
        for (int k = 0; k < num_vehicles_; ++k) {
            sprintf(buffer, "D_%02d", k+1);
            completion_[k].setName(buffer);
        }

        C_ = IloNumVarArray(env_, num_vehicles_, 0, IloInfinity);
        for (int k = 0; k < num_vehicles_; ++k) {
            sprintf(buffer, "C_%02d", k+1);
            C_[k].setName(buffer);
        }

        lu_ = IloArray<IloIntVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            lu_[k] = IloIntVarArray(env_, num_requests_ + 1, 0, 1);
            for (int i = 0; i <= num_requests_; ++i) {
                sprintf(buffer, "lu_%02d,%04d", k+1, i);

                lu_[k][i].setName(buffer);
            }
        }

        lr_ = IloArray<IloIntVarArray>(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            lr_[k] = IloIntVarArray(env_, num_requests_ + 1, 0, 1);
            for (int i = 0; i <= num_requests_; ++i) {
                sprintf(buffer, "lr_%02d,%04d", k+1, i);

                lr_[k][i].setName(buffer);
            }
        }

        u_ = IloIntVarArray(env_, num_vehicles_, 0, 1);
        for (int k = 0; k < num_vehicles_; ++k) {
            sprintf(buffer, "u_%02d", k+1);
            u_[k].setName(buffer);
        }

        r_ = IloIntVarArray(env_, num_vehicles_, 0, 1);
        for (int k = 0; k < num_vehicles_; ++k) {
            sprintf(buffer, "r_%02d", k+1);
            r_[k].setName(buffer);
        }

        if (input_settings_.obj_ == MAKESPAN_) {
            Cmax_ = IloNumVar(env_, 0, IloInfinity);
            Cmax_.setName("Cmax");
        }

    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }

}

void CompactModel::create_objective_function() {

    try {
        objective_ = IloAdd(master_, IloMinimize(env_));

        IloExpr obj(env_);

        for (int k = 0; k < num_vehicles_; ++k) {
            for (int i = first_customer_; i <= dest_; ++i) {
                if (i != dest_) {
                    obj += Instance::instance()->EdgeCost(0, i) * lambda_[k][x_index_[0][i]];
                }
                for (int j = i + 1; j <= dest_; ++j) {
                    obj += Instance::instance()->EdgeCost(i, j % dest_) * lambda_[k][x_index_[i][j]];
                }
            }
        }

        if (input_settings_.obj_ == MAKESPAN_) {
            obj += Cmax_;
        } else if (input_settings_.obj_ == TOTAL_COMPLETION_TIME_){
            for (int k = 0; k < num_vehicles_; ++k) {
                obj += C_[k];
            }
        }

        objective_.setExpr(obj);
        obj.end();

    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::calculate_big_Ms() {
    int vehicle_capacity = Instance::instance()->vehicle().capacity();
    multiset<int> demands;
    for (int i = 1; i <= num_suppliers_; ++i) {
        demands.insert(Instance::instance()->node(i).demand());
    }

    int max_nodes_per_vehicle = 0;
    int occupied = 0;
    for (multiset<int>::iterator it = demands.begin(); it != demands.end(); ++it) {
        if (occupied + *it <= vehicle_capacity) {
            ++max_nodes_per_vehicle;
            occupied += *it;
        }
    }

    double biggest_depot_arc = 0.0;
    double current_arc;
    for (int i = 1; i <= num_suppliers_; ++i) {
        current_arc = Instance::instance()->EdgeCost(0, i);
        if (cmp(current_arc, biggest_depot_arc) > 0) {
            biggest_depot_arc = current_arc;
        }
    }

    double bigM1_aux = 2 * biggest_depot_arc;

    multiset<double> non_depot_arcs;
    for (int i = 1; i <= num_suppliers_; ++i) {
        for (int j = i + 1; j <= num_suppliers_; ++j) {
            non_depot_arcs.insert(Instance::instance()->EdgeCost(i,j));
        }
    }

    int remaining_arcs = max_nodes_per_vehicle - 1;
    for (multiset<double>::reverse_iterator rit = non_depot_arcs.rbegin(); rit != non_depot_arcs.rend(), remaining_arcs > 0; ++rit, --remaining_arcs) {
        bigM1_aux += *rit;
    }

    multiset<int> unloading_times;
    for (int i = 1; i <= num_suppliers_; ++i) {
        unloading_times.insert(Instance::instance()->unloading_time(i));
    }

    int bigM2_aux = 0;
    int i = 0;
    for (multiset<int>::reverse_iterator rit = unloading_times.rbegin(); rit != unloading_times.rend(); ++rit, ++i) {
        if (i < max_nodes_per_vehicle - 1) {
            bigM1_aux += *rit;
        } else {
            bigM2_aux = *rit;
            break;
        }
    }

    bigM1_aux += Instance::instance()->unload_preparation_time();

    bigM1_ = (int)ceil(bigM1_aux);

    bigM2_aux += Instance::instance()->reload_preparation_time();
    bigM2_aux += Instance::instance()->between_docks_time(1, num_vehicles_);

    bigM2_ = bigM1_ + bigM2_aux;

    multiset<int> reloading_times;
    for (int i = 1; i <= num_requests_; ++i) {
        reloading_times.insert(Instance::instance()->reloading_time(i));
    }

    int bigM3_aux = bigM2_;
    int bigM4_aux = 0;
    i = 0;
    for (multiset<int>::reverse_iterator rit = reloading_times.rbegin(); rit != reloading_times.rend(); ++rit, ++i) {
        if (i < max_nodes_per_vehicle - 1) {
            bigM3_aux += *rit;
        } else {
            bigM4_aux = *rit;
            break;
        }
    }

    bigM3_ = bigM3_aux;
    bigM4_ = bigM3_ + bigM4_aux;

}

void CompactModel::create_constraints() {
    create_routing_constraints();
    create_delivery_routing_constraints();
    create_CD_constraints();
    create_reload_at_CD_constraints();
    if (input_settings_.obj_ == MAKESPAN_) {
        create_makespan_constraints();
    }
}

void CompactModel::create_routing_constraints() {
    try {
        // CONSTRAINT_2
        constraints_2_ = IloRangeArray(env_, num_vehicles_ * num_suppliers_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_suppliers_; ++i, ++constr_ind) {
                IloExpr expr(env_);

                for (int j = 0; j <= num_suppliers_; ++j) {
                    if (i != j) {
                        expr += f_[k][i][j] - f_[k][j][i];
                    }
                }
                expr += f_[k][i][dest_] - f_[k][dest_][i];

                expr += 2 * Instance::instance()->node(i).demand() * y_[k][i];

                constraints_2_[constr_ind] = expr == 0;

                char buffer[20];
                sprintf(buffer, "R2_%03d", constr_ind + 1);
                constraints_2_[constr_ind].setName(buffer);

                master_.add(constraints_2_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_3
        constraints_3_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            for (int i = 1; i <= num_suppliers_; ++i) {
                expr += f_[k][0][i] - y_[k][i] * Instance::instance()->node(i).demand();
            }

            constraints_3_[constr_ind] = expr == 0;

            char buffer[20];
            sprintf(buffer, "R3_%03d", constr_ind + 1);
            constraints_3_[constr_ind].setName(buffer);

            master_.add(constraints_3_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_4
        constraints_4_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            for (int i = 1; i <= num_suppliers_; ++i) {
                expr += f_[k][i][0] + y_[k][i] * Instance::instance()->node(i).demand();
            }

            constraints_4_[constr_ind] = expr == Instance::instance()->vehicle().capacity();

            char buffer[20];
            sprintf(buffer, "R4_%03d", constr_ind + 1);
            constraints_4_[constr_ind].setName(buffer);

            master_.add(constraints_4_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_5
        constraints_5_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            for (int i = 1; i <= num_suppliers_; ++i) {
                expr += f_[k][dest_][i];
            }

            constraints_5_[constr_ind] = expr == Instance::instance()->vehicle().capacity();

            char buffer[20];
            sprintf(buffer, "R5_%03d", constr_ind + 1);
            constraints_5_[constr_ind].setName(buffer);

            master_.add(constraints_5_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_6
        constraints_6_ = IloRangeArray(env_, num_vehicles_ * (num_edges_ / 2));
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 0; i <= num_suppliers_; ++i) {
                for (int j = i + 1; j <= num_suppliers_; ++j) {
                    IloExpr expr(env_);

                    expr += f_[k][i][j] + f_[k][j][i] - Instance::instance()->vehicle().capacity() * lambda_[k][x_index_[i][j]];

                    constraints_6_[constr_ind] = expr == 0;

                    char buffer[20];
                    sprintf(buffer, "R6_%03d", constr_ind + 1);
                    constraints_6_[constr_ind].setName(buffer);

                    master_.add(constraints_6_[constr_ind++]);
                    expr.end();
                }
                if (i != 0) {
                    IloExpr expr(env_);

                    expr += f_[k][i][dest_] + f_[k][dest_][i] - Instance::instance()->vehicle().capacity() * lambda_[k][x_index_[i][dest_]];

                    constraints_6_[constr_ind] = expr == 0;

                    char buffer[20];
                    sprintf(buffer, "R6_%03d", constr_ind + 1);
                    constraints_6_[constr_ind].setName(buffer);

                    master_.add(constraints_6_[constr_ind++]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_7
        constraints_7_ = IloRangeArray(env_, num_vehicles_ * num_suppliers_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_suppliers_; ++i, ++constr_ind) {
                IloExpr expr(env_);

                for (int j = 0; j <= num_suppliers_; ++j) {
                    if (i != j) {
                        expr += lambda_[k][x_index_[i][j]];
                    }
                }
                expr += lambda_[k][x_index_[i][dest_]];

                expr -= 2 * y_[k][i];

                constraints_7_[constr_ind] = expr == 0;

                char buffer[20];
                sprintf(buffer, "R7_%03d", constr_ind + 1);
                constraints_7_[constr_ind].setName(buffer);

                master_.add(constraints_7_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_8
        constraints_8_ = IloRangeArray(env_, num_suppliers_);
        for (int i = 1, constr_ind = 0; i <= num_suppliers_; ++i, ++constr_ind) {
            IloExpr expr(env_);

            for (int k = 0; k < num_vehicles_; ++k) {
                expr += y_[k][i];
            }

            constraints_8_[constr_ind] = expr == 1;

            char buffer[20];
            sprintf(buffer, "R8_%03d", constr_ind + 1);
            constraints_8_[constr_ind].setName(buffer);

            master_.add(constraints_8_[constr_ind]);
            expr.end();
        }
    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::create_delivery_routing_constraints() {
    try {
        // CONSTRAINT_9
        constraints_9_ = IloRangeArray(env_, num_vehicles_ * num_suppliers_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = first_customer_; i < dest_; ++i, ++constr_ind) {
                IloExpr expr(env_);

                for (int j = first_customer_; j <= dest_; ++j) {
                    if (i != j) {
                        expr += f_[k][i][j] - f_[k][j][i];
                    }
                }
                expr += f_[k][i][0] - f_[k][0][i];

                expr += 2 * Instance::instance()->node(i).demand() * y_[k][i];

                constraints_9_[constr_ind] = expr == 0;

                char buffer[20];
                sprintf(buffer, "R9_%03d", constr_ind + 1);
                constraints_9_[constr_ind].setName(buffer);

                master_.add(constraints_9_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_10
        constraints_10_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            for (int i = first_customer_; i < dest_; ++i) {
                expr += f_[k][0][i] - y_[k][i] * Instance::instance()->node(i).demand();
            }

            constraints_10_[constr_ind] = expr == 0;

            char buffer[20];
            sprintf(buffer, "R10_%03d", constr_ind + 1);
            constraints_10_[constr_ind].setName(buffer);

            master_.add(constraints_10_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_11
        constraints_11_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            for (int i = first_customer_; i < dest_; ++i) {
                expr += f_[k][i][0] + y_[k][i] * Instance::instance()->node(i).demand();
            }

            constraints_11_[constr_ind] = expr == Instance::instance()->vehicle().capacity();

            char buffer[20];
            sprintf(buffer, "R11_%03d", constr_ind + 1);
            constraints_11_[constr_ind].setName(buffer);

            master_.add(constraints_11_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_12
        constraints_12_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            for (int i = first_customer_; i < dest_; ++i) {
                expr += f_[k][dest_][i];
            }

            constraints_12_[constr_ind] = expr == Instance::instance()->vehicle().capacity();

            char buffer[20];
            sprintf(buffer, "R12_%03d", constr_ind + 1);
            constraints_12_[constr_ind].setName(buffer);

            master_.add(constraints_12_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_13
        constraints_13_ = IloRangeArray(env_, num_vehicles_ * (num_edges_ / 2));
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = first_customer_; i <= dest_; ++i) {
                for (int j = i + 1; j <= dest_; ++j) {
                    IloExpr expr(env_);

                    expr += f_[k][i][j] + f_[k][j][i] - Instance::instance()->vehicle().capacity() * lambda_[k][x_index_[i][j]];

                    constraints_13_[constr_ind] = expr == 0;

                    char buffer[20];
                    sprintf(buffer, "R13_%03d", constr_ind + 1);
                    constraints_13_[constr_ind].setName(buffer);

                    master_.add(constraints_13_[constr_ind++]);
                    expr.end();
                }
                if (i != dest_) {
                    IloExpr expr(env_);

                    expr += f_[k][i][0] + f_[k][0][i] - Instance::instance()->vehicle().capacity() * lambda_[k][x_index_[i][0]];

                    constraints_13_[constr_ind] = expr == 0;

                    char buffer[20];
                    sprintf(buffer, "R13_%03d", constr_ind + 1);
                    constraints_13_[constr_ind].setName(buffer);

                    master_.add(constraints_13_[constr_ind++]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_14
        constraints_14_ = IloRangeArray(env_, num_vehicles_ * num_suppliers_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = first_customer_; i < dest_; ++i, ++constr_ind) {
                IloExpr expr(env_);

                for (int j = first_customer_; j <= dest_; ++j) {
                    if (i != j) {
                        expr += lambda_[k][x_index_[i][j]];
                    }
                }
                expr += lambda_[k][x_index_[i][0]];

                expr -= 2 * y_[k][i];

                constraints_14_[constr_ind] = expr == 0;

                char buffer[20];
                sprintf(buffer, "R14_%03d", constr_ind + 1);
                constraints_14_[constr_ind].setName(buffer);

                master_.add(constraints_14_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_15
        constraints_15_ = IloRangeArray(env_, num_suppliers_);
        for (int i = first_customer_, constr_ind = 0; i < dest_; ++i, ++constr_ind) {
            IloExpr expr(env_);

            for (int k = 0; k < num_vehicles_; ++k) {
                expr += y_[k][i];
            }

            constraints_15_[constr_ind] = expr == 1;

            char buffer[20];
            sprintf(buffer, "R15_%03d", constr_ind + 1);
            constraints_15_[constr_ind].setName(buffer);

            master_.add(constraints_15_[constr_ind]);
            expr.end();
        }
    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::create_CD_constraints() {
    try {
        // CONSTRAINT_16
        constraints_16_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += lu_[k][i] - y_[k][i] + y_[k][i + num_requests_];

                constraints_16_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R16_%03d", constr_ind + 1);
                constraints_16_[constr_ind].setName(buffer);

                master_.add(constraints_16_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_17
        constraints_17_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += lu_[k][i] - y_[k][i];

                constraints_17_[constr_ind] = expr <= 0;

                char buffer[20];
                sprintf(buffer, "R17_%03d", constr_ind + 1);
                constraints_17_[constr_ind].setName(buffer);

                master_.add(constraints_17_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_18
        constraints_18_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += lu_[k][i] - (1 - y_[k][i + num_requests_]);

                constraints_18_[constr_ind] = expr <= 0;

                char buffer[20];
                sprintf(buffer, "R18_%03d", constr_ind + 1);
                constraints_18_[constr_ind].setName(buffer);

                master_.add(constraints_18_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_19
        constraints_19_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += u_[k] - lu_[k][i];

                constraints_19_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R19_%03d", constr_ind + 1);
                constraints_19_[constr_ind].setName(buffer);

                master_.add(constraints_19_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_20
        constraints_20_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += s_[k][i];

                for (int j = 0; j <= num_suppliers_; ++j) {
                    for (int l = j + 1; l <= num_suppliers_; ++l) {
                        expr -= Instance::instance()->EdgeCost(j, l) * lambda_[k][x_index_[j][l]];
                    }
                    if (j != 0) {
                        expr -= Instance::instance()->EdgeCost(j, 0) * lambda_[k][x_index_[j][dest_]];
                    }
                }

                expr -= Instance::instance()->unload_preparation_time() * u_[k];

                constraints_20_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R20_%03d", constr_ind + 1);
                constraints_20_[constr_ind].setName(buffer);

                master_.add(constraints_20_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_21
        constraints_21_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += z_[k][z_index_[i][j]] - lu_[k][i];

                    constraints_21_[constr_ind] = expr <= 0;

                    char buffer[20];
                    sprintf(buffer, "R21_%03d", constr_ind + 1);
                    constraints_21_[constr_ind].setName(buffer);

                    master_.add(constraints_21_[constr_ind]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_22
        constraints_22_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += z_[k][z_index_[i][j]] - lu_[k][j];

                    constraints_22_[constr_ind] = expr <= 0;

                    char buffer[20];
                    sprintf(buffer, "R22_%03d", constr_ind + 1);
                    constraints_22_[constr_ind].setName(buffer);

                    master_.add(constraints_22_[constr_ind]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_23
        constraints_23_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += s_[k][i] - s_[k][j]
                                - Instance::instance()->unloading_time(j)
                                + bigM1_ * z_[k][z_index_[i][j]]
                                + bigM1_ * (1 - lu_[k][i])
                                + bigM1_ * (1 - lu_[k][j]);

                    constraints_23_[constr_ind] = expr >= 0;

                    char buffer[20];
                    sprintf(buffer, "R23_%03d", constr_ind + 1);
                    constraints_23_[constr_ind].setName(buffer);

                    master_.add(constraints_23_[constr_ind]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_24
        constraints_24_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += s_[k][j] - s_[k][i]
                                - Instance::instance()->unloading_time(i)
                                + bigM1_ * (1 - z_[k][z_index_[i][j]]);

                    constraints_24_[constr_ind] = expr >= 0;

                    char buffer[20];
                    sprintf(buffer, "R24_%03d", constr_ind + 1);
                    constraints_24_[constr_ind].setName(buffer);

                    master_.add(constraints_24_[constr_ind]);
                    expr.end();
                }
            }
        }
    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::create_reload_at_CD_constraints() {
    try {
        // CONSTRAINT_25
        constraints_25_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += lr_[k][i] - y_[k][i + num_requests_] + y_[k][i];

                constraints_25_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R25_%03d", constr_ind + 1);
                constraints_25_[constr_ind].setName(buffer);

                master_.add(constraints_25_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_26
        constraints_26_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += lr_[k][i] - y_[k][i + num_requests_];

                constraints_26_[constr_ind] = expr <= 0;

                char buffer[20];
                sprintf(buffer, "R26_%03d", constr_ind + 1);
                constraints_26_[constr_ind].setName(buffer);

                master_.add(constraints_26_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_27
        constraints_27_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += lr_[k][i] - (1 - y_[k][i]);

                constraints_27_[constr_ind] = expr <= 0;

                char buffer[20];
                sprintf(buffer, "R27_%03d", constr_ind + 1);
                constraints_27_[constr_ind].setName(buffer);

                master_.add(constraints_27_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_28
        constraints_28_ = IloRangeArray(env_, num_requests_ * num_vehicles_);
        for (int i = 1, constr_ind = 0; i <= num_requests_; ++i) {
            for (int k = 0; k < num_vehicles_; ++k, ++constr_ind) {
                IloExpr expr(env_);

                expr += r_[k] - lr_[k][i];

                constraints_28_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R28_%03d", constr_ind + 1);
                constraints_28_[constr_ind].setName(buffer);

                master_.add(constraints_28_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_29
        constraints_29_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            expr += completion_[k];

            for (int i = 0; i <= num_suppliers_; ++i) {
                for (int j = i + 1; j <= num_suppliers_; ++j) {
                    expr -= Instance::instance()->EdgeCost(i, j) * lambda_[k][x_index_[i][j]];
                }
                if (i != 0) {
                    expr -= Instance::instance()->EdgeCost(i, 0) * lambda_[k][x_index_[i][dest_]];
                }
            }

            expr -= Instance::instance()->reload_preparation_time() * r_[k];

            constraints_29_[constr_ind] = expr >= 0;

            char buffer[20];
            sprintf(buffer, "R29_%03d", constr_ind + 1);
            constraints_29_[constr_ind].setName(buffer);

            master_.add(constraints_29_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_30
        constraints_30_ = IloRangeArray(env_, num_vehicles_ * num_requests_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i, ++constr_ind) {
                IloExpr expr(env_);

                expr += completion_[k] - s_[k][i]
                           - Instance::instance()->unloading_time(i)
                           - Instance::instance()->reload_preparation_time() * r_[k]
                           + bigM2_ * (1 - lu_[k][i]);

                constraints_30_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R30_%03d", constr_ind + 1);
                constraints_30_[constr_ind].setName(buffer);

                master_.add(constraints_30_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_31
        constraints_31_ = IloRangeArray(env_, num_vehicles_ * num_requests_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i, ++constr_ind) {
                IloExpr expr(env_);

                expr += sr_[k][i] - completion_[k];

                constraints_31_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R31_%03d", constr_ind + 1);
                constraints_31_[constr_ind].setName(buffer);

                master_.add(constraints_31_[constr_ind]);
                expr.end();
            }
        }

        // CONSTRAINT_32
        constraints_32_ = IloRangeArray(env_, num_vehicles_ * (num_vehicles_ - 1) * num_requests_);
        for (int k1 = 0, constr_ind = 0; k1 < num_vehicles_; ++k1) {
            for (int k2 = 0; k2 < num_vehicles_; ++k2) {
                if (k1 != k2) {
                    for (int i = 1; i <= num_requests_; ++i, ++constr_ind) {
                        IloExpr expr(env_);

                        expr += sr_[k1][i] - s_[k2][i]
                                    - Instance::instance()->unloading_time(i)
                                    - Instance::instance()->between_docks_time(k2, k1)
                                    + bigM2_ * (1 - lu_[k2][i]);

                        constraints_32_[constr_ind] = expr >= 0;

                        char buffer[20];
                        sprintf(buffer, "R32_%03d", constr_ind + 1);
                        constraints_32_[constr_ind].setName(buffer);

                        master_.add(constraints_32_[constr_ind]);
                        expr.end();
                    }
                }
            }
        }

        // CONSTRAINT_33
        constraints_33_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += zr_[k][z_index_[i][j]] - lr_[k][i];

                    constraints_33_[constr_ind] = expr <= 0;

                    char buffer[20];
                    sprintf(buffer, "R33_%03d", constr_ind + 1);
                    constraints_33_[constr_ind].setName(buffer);

                    master_.add(constraints_33_[constr_ind]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_34
        constraints_34_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += zr_[k][z_index_[i][j]] - lr_[k][j];

                    constraints_34_[constr_ind] = expr <= 0;

                    char buffer[20];
                    sprintf(buffer, "R34_%03d", constr_ind + 1);
                    constraints_34_[constr_ind].setName(buffer);

                    master_.add(constraints_34_[constr_ind]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_35
        constraints_35_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k1 = 0, constr_ind = 0; k1 < num_vehicles_; ++k1) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += sr_[k1][i] - sr_[k1][j]
                                - Instance::instance()->reloading_time(j)
                                + bigM3_ * zr_[k1][z_index_[i][j]]
                                + bigM3_ * (1 - lr_[k1][i])
                                + bigM3_ * (1 - lr_[k1][j]);

                    constraints_35_[constr_ind] = expr >= 0;

                    char buffer[20];
                    sprintf(buffer, "R35_%03d", constr_ind + 1);
                    constraints_35_[constr_ind].setName(buffer);

                    master_.add(constraints_35_[constr_ind]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_36
        constraints_36_ = IloRangeArray(env_, num_vehicles_ * (num_requests_ * (num_requests_ - 1)) / 2);
        for (int k1 = 0, constr_ind = 0; k1 < num_vehicles_; ++k1) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j, ++constr_ind) {
                    IloExpr expr(env_);

                    expr += sr_[k1][j] - sr_[k1][i]
                                - Instance::instance()->reloading_time(i)
                                + bigM3_ * (1 - zr_[k1][z_index_[i][j]]);

                    constraints_36_[constr_ind] = expr >= 0;

                    char buffer[20];
                    sprintf(buffer, "R36_%03d", constr_ind + 1);
                    constraints_36_[constr_ind].setName(buffer);

                    master_.add(constraints_36_[constr_ind]);
                    expr.end();
                }
            }
        }

        // CONSTRAINT_37
        constraints_37_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0, constr_ind = 0; k < num_vehicles_; ++k, ++constr_ind) {
            IloExpr expr(env_);

            expr += C_[k] - completion_[k];

            constraints_37_[constr_ind] = expr >= 0;

            char buffer[20];
            sprintf(buffer, "R37_%03d", constr_ind + 1);
            constraints_37_[constr_ind].setName(buffer);

            master_.add(constraints_37_[constr_ind]);
            expr.end();
        }

        // CONSTRAINT_38
        constraints_38_ = IloRangeArray(env_, num_vehicles_ * num_requests_);
        for (int k1 = 0, constr_ind = 0; k1 < num_vehicles_; ++k1) {
            for (int i = 1; i <= num_requests_; ++i, ++constr_ind) {
                IloExpr expr(env_);

                expr += C_[k1] - sr_[k1][i]
                            - Instance::instance()->reloading_time(i)
                            + bigM4_ * (1 - lr_[k1][i]);

                constraints_38_[constr_ind] = expr >= 0;

                char buffer[20];
                sprintf(buffer, "R38_%03d", constr_ind + 1);
                constraints_38_[constr_ind].setName(buffer);

                master_.add(constraints_38_[constr_ind]);
                expr.end();
            }
        }
    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::create_makespan_constraints() {
    try {
        constraints_Cmax_ = IloRangeArray(env_, num_vehicles_);
        for (int k = 0; k < num_vehicles_; ++k) {
            IloExpr expr(env_);
            expr += Cmax_ - C_[k];

            constraints_Cmax_[k] =  expr >= 0;
            char buffer[20];
            sprintf(buffer, "RCmax_%03d", k + 1);
            constraints_Cmax_[k].setName(buffer);

            master_.add(constraints_Cmax_[k]);
        }
    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::turn_off_parameters() {

    try {
        master_cpx_->setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        master_cpx_->setParam(IloCplex::Param::Preprocessing::Aggregator    , 0);
        master_cpx_->setParam(IloCplex::Param::MIP::Strategy::HeuristicFreq    , -1);

        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::Gomory, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::FlowCovers, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::GUBCovers, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::Covers, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::MCFCut, -1);
        //versao 11
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::ZeroHalfCut, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::Implied, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::Cliques, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::Disjunctive, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::PathCut, -1);
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::MIRCut, -1);

        // CPLEX 12.6
        master_cpx_->setParam(IloCplex::Param::MIP::Cuts::LiftProj, -1);

        master_cpx_->setParam(IloCplex::Param::MIP::Limits::CutsFactor, 0);
    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }

}

void CompactModel::printSolutionOnFile(const char * solution_file_name) {

    try {
        FILE * sol_file;
        double cost, total_s_cost = 0.0, total_c_cost = 0.0, fo = 0.0;
        int current_node, load;
        vector<vector<bool> > sup_visited_nodes_in_route, cus_visited_nodes_in_route;
        vector<bool> sup_visited_nodes, cus_visited_nodes;

        sol_file = fopen(solution_file_name, "w");

        char route[200];

        // Rotas de coleta
        sup_visited_nodes_in_route.resize(num_vehicles_);
        sup_visited_nodes.resize(num_suppliers_ + 2, false);
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "           SUPPLIERS                                            \n");
        fprintf(sol_file, "%3s %8s     %4s    %s\n", "k", "cost", "load", "route");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            cost = 0.0;
            fprintf(sol_file, "%3d ", k+1);
            sprintf(route, "(0");
            current_node = 0;
            load = 0;
            sup_visited_nodes_in_route[k].resize(num_suppliers_ + 1, false);

            for (int j = 0; j <= num_suppliers_; ++j) {
                if (!sup_visited_nodes[j]
                        && current_node != j
                        && cmp(master_cpx_->getValue(lambda_[k][x_index_[current_node][j]]), 0.5) > 0) {

                    sprintf(route + strlen(route), "-%d", j);
                    cost += Instance::instance()->EdgeCost(current_node /*% (num_real_nodes_)*/, j);
                    current_node = j;
                    load += Instance::instance()->node(j).demand();
                    sup_visited_nodes[j] = sup_visited_nodes_in_route[k][j] = true;
                    j = 0;
                }
            }
            if (cmp(master_cpx_->getValue(lambda_[k][x_index_[current_node][dest_]]), 0.5) > 0) {
                sprintf(route + strlen(route), "-0)");
                cost += Instance::instance()->EdgeCost(current_node, 0);
            }

            if (load > Instance::instance()->vehicle().capacity()) {
                cerr << "ERRO! Rota com carregamento inviavel: " << route << " | Load = " << load << endl;
            }
            fprintf(sol_file, "%11.3lf    %2d    %s\n", cost, load, route);
            total_s_cost += cost;
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "%4s %10.3lf\n", "tot", total_s_cost);
        fprintf(sol_file, "----------------------------------------------------------------\n");

        // Momento que o veículo está disponível para partir do CD após a consolidação
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              ready_k                                           \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            fprintf(sol_file, "%3d      ", k+1);
            fprintf(sol_file, "%12.3lf\n", master_cpx_->getValue(completion_[k]));
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");

        // Momento que o veículo está disponível para partir do CD após a consolidação
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              completion_k                                      \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            fprintf(sol_file, "%3d      ", k+1);
            fprintf(sol_file, "%12.3lf\n", master_cpx_->getValue(C_[k]));
            if (input_settings_.obj_ == TOTAL_COMPLETION_TIME_) {
                fo += master_cpx_->getValue(C_[k]);
            }
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");

        // MAKESPAN
        if (input_settings_.obj_ == MAKESPAN_) {
            fprintf(sol_file, "----------------------------------------------------------------\n");
            fprintf(sol_file, "              MAKESPAN                                          \n");
            fprintf(sol_file, "----------------------------------------------------------------\n");
            fprintf(sol_file, "Cmax      %12.3lf\n", master_cpx_->getValue(Cmax_));
            fo += master_cpx_->getValue(Cmax_);
            fprintf(sol_file, "----------------------------------------------------------------\n");
        }

        // Rotas de entrega
        cus_visited_nodes_in_route.resize(num_vehicles_);
        cus_visited_nodes.resize(num_suppliers_ + 2, false);
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "           CUSTOMERS                                            \n");
        fprintf(sol_file, "%3s %8s     %4s    %s\n", "k", "cost", "load", "route");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            cost = 0.0;
            fprintf(sol_file, "%3d ", k+1);
            sprintf(route, "(0");
            current_node = 0;
            load = 0;
            cus_visited_nodes_in_route[k].resize(num_suppliers_ + 1, false);

            for (int j = first_customer_; j <= num_real_nodes_; ++j) {
                if (!cus_visited_nodes[(j - num_suppliers_) % first_customer_]
                        && current_node != j % num_real_nodes_
                        && cmp(master_cpx_->getValue(lambda_[k][x_index_[current_node][j]]), 0.5) > 0) {

                    sprintf(route + strlen(route), "-%d", (j - num_suppliers_) % first_customer_);
                    cost += Instance::instance()->EdgeCost(current_node, j % num_real_nodes_);
                    current_node = j;
                    if (j % num_real_nodes_ != 0) {
                        load += Instance::instance()->node(j).demand();
                        cus_visited_nodes[j - num_suppliers_] = cus_visited_nodes_in_route[k][j - num_suppliers_] = true;
                        j = first_customer_ - 1;
                    } else {
                        break;
                    }
                }
            }

            sprintf(route + strlen(route), ")");
            if (load > Instance::instance()->vehicle().capacity()) {
                cerr << "ERRO! Rota com carregamento inviavel: " << route << " | Load = " << load << endl;
            }
            fprintf(sol_file, "%11.3lf    %2d    %s\n", cost, load, route);
            total_c_cost += cost;
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "%4s %10.3lf\n", "tot", total_c_cost);
        fprintf(sol_file, "----------------------------------------------------------------\n");

        fo += total_c_cost;
        // Impressao do valor da funcao objetivo
        fprintf(sol_file, "\n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "%4s %10.3lf\n", "FO=", fo);
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "\n");

        // Impressao dos valores da variavel y
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              y                                                   \n");
        fprintf(sol_file, " k  \\  i=");
        for (int i = 1; i <= num_suppliers_; ++i) fprintf(sol_file, "%5d", i);
        fprintf(sol_file, "\n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            fprintf(sol_file, "%2d       ", k);
            for (int i = 1; i <= num_requests_; ++i) {
                fprintf(sol_file, "%5.0lf", fabs(master_cpx_->getValue(y_[k][i])));
            }
            fprintf(sol_file, "\n");
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");


        // Impressao dos valores da variavel z
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              z                                                   \n");
        fprintf(sol_file, " k  \\  i=");
        for (int j = 1; j <= num_customers_; ++j) fprintf(sol_file, "%5d", j);
        fprintf(sol_file, "\n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            fprintf(sol_file, "%2d       ", k);
            for (int j = first_customer_; j < num_real_nodes_; ++j) {
                fprintf(sol_file, "%5.0lf", fabs(master_cpx_->getValue(y_[k][j])));
            }
            fprintf(sol_file, "\n");
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");


        // Impressao dos valores da variavel un
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              un                                                   \n");
        fprintf(sol_file, " (  k,  i)                                                        \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                if (cmp(master_cpx_->getValue(lu_[k][i]), 0.5) > 0) {
                    fprintf(sol_file, " (%3d,%3d):      ", k+1, i);
                    fprintf(sol_file, "%12.6lf\n", master_cpx_->getValue(lu_[k][i]));
                }
            }
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");

        // Impressao dos valores da variavel u
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              u                                                    \n");
        fprintf(sol_file, " (  k)                                                          \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            if (cmp(master_cpx_->getValue(u_[k]), 0.5) > 0) {
                fprintf(sol_file, " (%3d):      ", k+1);
                fprintf(sol_file, "%12.6lf\n", master_cpx_->getValue(u_[k]));
            }
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");


        // Impressao dos valores da variavel re
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              re                                                   \n");
        fprintf(sol_file, " (  k,  i)                                                        \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                if (cmp(master_cpx_->getValue(lr_[k][i]), 0.5) > 0) {
                    fprintf(sol_file, " (%3d,%3d):      ", k+1, i);
                    fprintf(sol_file, "%12.6lf\n", master_cpx_->getValue(lr_[k][i]));
                }
            }
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");

        // Impressao dos valores da variavel r
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              r                                                    \n");
        fprintf(sol_file, " (  k)                                                          \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            if (cmp(master_cpx_->getValue(r_[k]), 0.5) > 0) {
                fprintf(sol_file, " (%3d):      ", k+1);
                fprintf(sol_file, "%12.6lf\n", master_cpx_->getValue(r_[k]));
            }
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");


        // Impressao dos valores da variavel su
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              su                                                   \n");
        fprintf(sol_file, "  i  \\  k=");
        for (int k = 1; k <= num_vehicles_; ++k) fprintf(sol_file, "%12d", k);
        fprintf(sol_file, "\n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int r = 1; r <= num_requests_; ++r) {
            fprintf(sol_file, " %2d       ", r);
            for (int k = 0; k < num_vehicles_; ++k) {
                fprintf(sol_file, "%12.3lf", master_cpx_->getValue(s_[k][r]));
            }
            fprintf(sol_file, "\n");
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");


        // Impressao dos valores da variavel sr
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              sr                                                   \n");
        fprintf(sol_file, "  i  \\  k=");
        for (int k = 1; k <= num_vehicles_; ++k) fprintf(sol_file, "%12d", k);
        fprintf(sol_file, "\n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int r = 1; r <= num_requests_; ++r) {
            fprintf(sol_file, " %2d       ", r);
            for (int k = 0; k < num_vehicles_; ++k) {
                fprintf(sol_file, "%12.3lf", master_cpx_->getValue(sr_[k][r]));
            }
            fprintf(sol_file, "\n");
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");


        // Impressao dos valores da variavel zu
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              zu                                                   \n");
        fprintf(sol_file, " (  k,  i,  j)                                                     \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j) {
                    if (cmp(master_cpx_->getValue(z_[k][z_index_[i][j]]), 0.5) > 0) {
                        fprintf(sol_file, " (%3d,%3d,%3d):      ", k+1, i, j);
                        fprintf(sol_file, "%12.3lf\n", master_cpx_->getValue(z_[k][z_index_[i][j]]));
                    }
                }
            }
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");


        // Impressao dos valores da variavel zr
        fprintf(sol_file, "----------------------------------------------------------------\n");
        fprintf(sol_file, "              zr                                                   \n");
        fprintf(sol_file, " (  k,  i,  j)                                                     \n");
        fprintf(sol_file, "----------------------------------------------------------------\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                for (int j = i + 1; j <= num_requests_; ++j) {
                    if (cmp(master_cpx_->getValue(zr_[k][z_index_[i][j]]), 0.5) > 0) {
                        fprintf(sol_file, " (%3d,%3d,%3d):      ", k+1, i, j);
                        fprintf(sol_file, "%12.3lf\n", master_cpx_->getValue(zr_[k][z_index_[i][j]]));
                    }
                }
            }
        }
        fprintf(sol_file, "----------------------------------------------------------------\n");

        fclose(sol_file);

    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }

}

void CompactModel::printTikZSolutionGuide(const char * tikz_guide_file_name) {
    try {
        FILE * tikz_file = fopen(tikz_guide_file_name, "w");

        vector<bool> do_unload(num_vehicles_, false), do_reload(num_vehicles_, false);
        for(int k = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                if (cmp(master_cpx_->getValue(lu_[k][i]), 0.5) > 0) {
                    do_unload[k] = true;
                    break;
                }
            }
        }
        for(int k = 0; k < num_vehicles_; ++k) {
            for (int i = 1; i <= num_requests_; ++i) {
                if (cmp(master_cpx_->getValue(lr_[k][i]), 0.5) > 0) {
                    do_reload[k] = true;
                    break;
                }
            }
        }

        fprintf(tikz_file, "INSTANCE_NAME: %s\n", Instance::instance()->name());
        fprintf(tikz_file, "NUM_REQUESTS: %d\n", num_requests_);
        fprintf(tikz_file, "NUM_VEHICLES: %d\n", num_vehicles_);
        fprintf(tikz_file, "SOLUTION_COST: %.3lf\n", master_cpx_->getObjValue());
        if (input_settings_.obj_ == MAKESPAN_) {
            fprintf(tikz_file, "CRITERION: makespan\n\n");
        } else if (input_settings_.obj_ == TOTAL_COMPLETION_TIME_) {
            fprintf(tikz_file, "CRITERION: total completion time\n\n");
        }

        fprintf(tikz_file, "COORDINATES(SUPPLIER-CUSTOMER)\n");
        fprintf(tikz_file, "%3d: %10.1lf %10.1lf\n", 0, Instance::instance()->node(0).x(), Instance::instance()->node(0).y());
        for (int i = 1; i <= num_requests_; ++i) {
            fprintf(tikz_file, "%3d: %10.1lf %10.1lf %10.1lf %10.1lf %10d\n", i, Instance::instance()->node(i).x(), Instance::instance()->node(i).y(),
                    Instance::instance()->node(i+num_requests_).x(), Instance::instance()->node(i+num_requests_).y(), Instance::instance()->node(i).demand());
        }

        fprintf(tikz_file, "\nROUTES_PICKUP\n");

        double start_scheduling = 1000000000.0;
        vector<double> costs_pic(num_vehicles_, 0.0);
        char mount_route[200] = {};

        int current_node, previous_node, load;
        for (int k = 0; k < num_vehicles_; ++k) {
            fprintf(tikz_file, "k%d:", k+1);
            sprintf(mount_route, "(0");
            previous_node = -1;
            current_node = 0;
            load = 0;

            for (int j = 0; j <= num_suppliers_; ++j) {
                if (previous_node != j
                        && current_node != j
                        && cmp(master_cpx_->getValue(lambda_[k][x_index_[current_node][j]]), 0.5) > 0) {

                    previous_node = current_node;
                    current_node = j;
                    j = 0;

                    sprintf(mount_route + strlen(mount_route), "%5d", current_node);
                    costs_pic[k] += Instance::instance()->EdgeCost(previous_node, current_node);
                    load += Instance::instance()->node(current_node).demand();
                }
            }
            if (cmp(master_cpx_->getValue(lambda_[k][x_index_[current_node][dest_]]), 0.5) > 0) {
                sprintf(mount_route + strlen(mount_route), "%5d)", 0);
                costs_pic[k] += Instance::instance()->EdgeCost(current_node, 0);
            }

            if (cmp(costs_pic[k], start_scheduling) < 0 && (do_unload[k] || do_reload[k])) {
                start_scheduling = costs_pic[k];
            }

            fprintf(tikz_file, "    C: %8.3lf    L: %3d    %s\n", costs_pic[k], load, mount_route);
        }

        fprintf(tikz_file, "\nROUTES_DELIVERY\n");

        for (int k = 0; k < num_vehicles_; ++k) {
            double cost = 0.0;
            fprintf(tikz_file, "k%d:", k+1);
            sprintf(mount_route, "(0");
            previous_node = -1;
            current_node = 0;
            load = 0;

            for (int j = first_customer_; j < num_real_nodes_; ++j) {
                if (previous_node != j
                        && current_node != j
                        && cmp(master_cpx_->getValue(lambda_[k][x_index_[current_node][j]]), 0.5) > 0) {

                    previous_node = current_node;
                    current_node = j;
                    j = first_customer_ - 1;

                    sprintf(mount_route + strlen(mount_route), "%5d", current_node - num_requests_);
                    cost += Instance::instance()->EdgeCost(previous_node, current_node);
                    load += Instance::instance()->node(current_node).demand();
                }
            }
            if (cmp(master_cpx_->getValue(lambda_[k][x_index_[current_node][dest_]]), 0.5) > 0) {
                sprintf(mount_route + strlen(mount_route), "%5d)", 0);
                cost += Instance::instance()->EdgeCost(current_node, 0);
            }

            fprintf(tikz_file, "    C: %8.3lf    L: %3d    %s\n", cost, load, mount_route);
        }

        fprintf(tikz_file, "\nSCHEDULING\n");

        double makespan = 0.0;
        for (int k = 0; k < num_vehicles_; ++k) {
            if (cmp(makespan, master_cpx_->getValue(C_[k])) < 0 && (do_unload[k] || do_reload[k])) {
                makespan = master_cpx_->getValue(C_[k]);
            }
        }

        fprintf(tikz_file, "START_SCHEDULING: %.3lf\n", (start_scheduling == 1000000000.0 ? -1: start_scheduling));
        fprintf(tikz_file, "MAKESPAN: %.3lf\n\n", makespan);

        fprintf(tikz_file, "PROCESSING_TIMES\n");
        for (int k = 0; k < num_vehicles_; ++k) {
            if (cmp(costs_pic[k] - start_scheduling, 0.0) > 0 && (do_unload[k] || do_reload[k])) {
                fprintf(tikz_file, "T %5d %14.3lf %8.3lf\n", k+1, start_scheduling, costs_pic[k] - start_scheduling);
            }
        }

        for (int k = 0; k < num_vehicles_; ++k) {
            if (do_unload[k]) {
                for (int i = 1; i <= num_requests_; ++i) {
                    if (cmp(master_cpx_->getValue(lu_[k][i]), 0.5) > 0) {
                        fprintf(tikz_file, "U %5d %5d %8.3lf %8d\n", k+1, i, master_cpx_->getValue(s_[k][i]), Instance::instance()->unloading_time(i));
                    }
                }
            }
        }

        for (int k = 0; k < num_vehicles_; ++k) {
            if (do_reload[k]) {
                for (int i = 1; i <= num_requests_; ++i) {
                    if (cmp(master_cpx_->getValue(lr_[k][i]), 0.5) > 0) {
                        fprintf(tikz_file, "R %5d %5d %8.3lf %8d\n", k+1, i, master_cpx_->getValue(sr_[k][i]), Instance::instance()->reloading_time(i));
                    }
                }
            }
        }

        for (int k = 0; k < num_vehicles_; ++k) {
            if (do_unload[k]) {
                fprintf(tikz_file, "P %5d %14.3lf %8.3lf\n", k+1, costs_pic[k], (double)Instance::instance()->unload_preparation_time());
            }
        }

        for (int k = 0; k < num_vehicles_; ++k) {
            if (do_reload[k]) {
                if (do_unload[k]) {
                    double start_prep = 0.0;
                    for (int i = 1; i <= num_requests_; ++i) {
                        if (cmp(master_cpx_->getValue(lu_[k][i]), 0.5) > 0 && cmp(master_cpx_->getValue(s_[k][i]) + Instance::instance()->unloading_time(i), start_prep) > 0) {
                            start_prep = master_cpx_->getValue(s_[k][i]) + Instance::instance()->unloading_time(i);
                        }
                    }
                    fprintf(tikz_file, "P %5d %14.3lf %8.3lf\n", k+1, start_prep, (double)Instance::instance()->reload_preparation_time());
                } else {
                    fprintf(tikz_file, "P %5d %14.3lf %8.3lf\n", k+1, costs_pic[k], (double)Instance::instance()->reload_preparation_time());
                }
            }
        }

        fclose(tikz_file);

    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::printTikZRelaxationGuide(const char * tikz_relaxation_file_name) {
    try {
        FILE * tikz_file = fopen(tikz_relaxation_file_name, "w");

        fprintf(tikz_file, "INSTANCE_NAME: %s\n", Instance::instance()->name());
        fprintf(tikz_file, "NUM_REQUESTS: %d\n", num_requests_);
        fprintf(tikz_file, "NUM_VEHICLES: %d\n", num_vehicles_);
        fprintf(tikz_file, "RELAXATION_COST: %.3lf\n", master_cpx_->getObjValue());
        if (input_settings_.obj_ == MAKESPAN_) {
            fprintf(tikz_file, "CRITERION: makespan\n\n");
        } else if (input_settings_.obj_ == TOTAL_COMPLETION_TIME_) {
            fprintf(tikz_file, "CRITERION: total completion time\n\n");
        }

        fprintf(tikz_file, "COORDINATES(SUPPLIER-CUSTOMER)\n");
        fprintf(tikz_file, "%3d: %10.1lf %10.1lf\n", 0, Instance::instance()->node(0).x(), Instance::instance()->node(0).y());
        for (int i = 1; i <= num_requests_; ++i) {
            fprintf(tikz_file, "%3d: %10.1lf %10.1lf %10.1lf %10.1lf %10d\n", i, Instance::instance()->node(i).x(), Instance::instance()->node(i).y(),
                    Instance::instance()->node(i+num_requests_).x(), Instance::instance()->node(i+num_requests_).y(), Instance::instance()->node(i).demand());
        }

        for (int k = 0; k < num_vehicles_; ++k) {
            fprintf(tikz_file, "K %d\n", k+1);

            for (int i = orig_; i <= num_suppliers_; ++i) {
                for (int j = i+1; j <= num_suppliers_; ++j) {
                    if (cmp(master_cpx_->getValue(lambda_[k][x_index_[i][j]]), 0.0) > 0) {
                        fprintf(tikz_file, "X %3d %3d %8.3lf\n", i, j, master_cpx_->getValue(lambda_[k][x_index_[i][j]]));
                    }
                }
                if (i != orig_ && cmp(master_cpx_->getValue(lambda_[k][x_index_[i][dest_]]), 0.0) > 0) {
                    fprintf(tikz_file, "X %3d %3d %8.3lf\n", i, 0, master_cpx_->getValue(lambda_[k][x_index_[i][dest_]]));
                }
            }

            for (int i = first_customer_; i <= last_customer_; ++i) {
                for (int j = i+1; j <= dest_; ++j) {
                    if (cmp(master_cpx_->getValue(lambda_[k][x_index_[i][j]]), 0.0) > 0) {
                        fprintf(tikz_file, "X %3d %3d %8.3lf\n", i, j % dest_, master_cpx_->getValue(lambda_[k][x_index_[i][j]]));
                    }
                }
                if (cmp(master_cpx_->getValue(lambda_[k][x_index_[0][i]]), 0.0) > 0) {
                    fprintf(tikz_file, "X %3d %3d %8.3lf\n", 0, i, master_cpx_->getValue(lambda_[k][x_index_[0][i]]));
                }
            }
        }

        fclose(tikz_file);

    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }
}

void CompactModel::printReport() {

    string criterion;
    if (input_settings_.obj_ == TOTAL_COMPLETION_TIME_) {
        criterion = "TCT";
    } else if (input_settings_.obj_ == MAKESPAN_) {
        criterion = "MKP";
    }

    if (input_settings_.only_relaxation_) {
        stringstream overall_status;
        overall_status << report_infos_.overall_status_;

        printf("%-17s; %5s; %2d; %10.3lf; %12s; %10.4lf\n",
             Instance::instance()->name(), criterion.c_str(), report_infos_.num_threads_,
             report_infos_.overall_dual_obj_value_, overall_status.str().c_str(), report_infos_.overall_time_);
    } else {
        int num_vehicle_swaps = 0;
        try {
            for (int i = 1; i <= num_suppliers_; ++i) {
                for (int k = 0; k < num_vehicles_; ++k) {
                    if (cmp(master_cpx_->getValue(lu_[k][i]), 0.5) > 0) {
                        ++num_vehicle_swaps;
                        break;
                    }
                }
            }
        } catch (IloException& e) {
            cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
        } catch (...) {
            cerr << "Error" << endl;
        }

        stringstream root_status, overall_status;
        root_status << report_infos_.root_status_;
        overall_status << report_infos_.overall_status_;

        printf("%-17s; %5s; %2d; %10.3lf; %12s; %10.4lf; %10.3lf; %10.3lf; %12s; %7.3lf; %10.4lf; %10d; %3d\n",
             Instance::instance()->name(), criterion.c_str(), report_infos_.num_threads_, report_infos_.root_dual_obj_value_,
             root_status.str().c_str(), report_infos_.root_time_, report_infos_.overall_dual_obj_value_,
             report_infos_.overall_int_obj_value_, overall_status.str().c_str(), report_infos_.gap_ * 100.0,
             report_infos_.overall_time_, report_infos_.num_tree_nodes_, num_vehicle_swaps);
    }
}

void CompactModel::solve() {

    try {
        create_model();

        ofstream cplex_log(input_settings_.log_file_name_);
        master_cpx_->setOut(cplex_log);
        master_cpx_->setWarning(cplex_log);

        master_cpx_->setParam(IloCplex::Param::Threads, input_settings_.num_threads_);

        if (input_settings_.only_relaxation_) {
            double ini_time = master_cpx_->getCplexTime();
            for (int k = 0; k < num_vehicles_; ++k) {
                master_.add(IloConversion(env_, lambda_[k], ILOFLOAT));
                master_.add(IloConversion(env_, y_[k], ILOFLOAT));
                master_.add(IloConversion(env_, z_[k], ILOFLOAT));
                master_.add(IloConversion(env_, zr_[k], ILOFLOAT));
                master_.add(IloConversion(env_, lu_[k], ILOFLOAT));
                master_.add(IloConversion(env_, lr_[k], ILOFLOAT));
            }
            master_.add(IloConversion(env_, u_, ILOFLOAT));
            master_.add(IloConversion(env_, r_, ILOFLOAT));

            master_cpx_->solve();

            report_infos_.overall_time_ = master_cpx_->getCplexTime() - ini_time;
            report_infos_.overall_dual_obj_value_ = master_cpx_->getObjValue();
            report_infos_.overall_status_ = master_cpx_->getStatus();
        } else {
            // limite de nos default para ser re-setado apos a resolucao da raiz
            long cplexNodeLim = master_cpx_->getParam(IloCplex::Param::MIP::Limits::Nodes);

            // Solving the root
            double ini_root_time = master_cpx_->getCplexTime();
            master_cpx_->setParam(IloCplex::Param::TimeLimit, input_settings_.time_limit_);
            master_cpx_->setParam(IloCplex::Param::MIP::Limits::Nodes, 0);

            master_cpx_->solve();

            report_infos_.root_time_ = master_cpx_->getCplexTime() - ini_root_time;
            report_infos_.root_status_ = master_cpx_->getStatus();
            report_infos_.root_dual_obj_value_ = master_cpx_->getBestObjValue();
            // Solving the B&B
            double remaining_time = input_settings_.time_limit_ - report_infos_.root_time_;
            if (cmp(remaining_time, 0.0) > 0) {
                double ini_bb_time = master_cpx_->getCplexTime();
                master_cpx_->setParam(IloCplex::Param::TimeLimit, remaining_time);
                master_cpx_->setParam(IloCplex::Param::MIP::Limits::Nodes, cplexNodeLim);

                if (master_cpx_->solve()) {
                    report_infos_.overall_int_obj_value_ = master_cpx_->getObjValue();
                    report_infos_.gap_ = master_cpx_->getMIPRelativeGap();
                }
                report_infos_.overall_time_ = master_cpx_->getCplexTime() - ini_bb_time + report_infos_.root_time_;
                report_infos_.overall_dual_obj_value_ = master_cpx_->getBestObjValue();
                report_infos_.overall_status_ = master_cpx_->getStatus();
                report_infos_.num_tree_nodes_ = master_cpx_->getNnodes();
            }
        }

        cplex_log.close();

    } catch (IloException& e) {
        cerr << "ERROR (at method " << __FUNCTION__ << "): " << e.getMessage() << endl;
    } catch (...) {
        cerr << "Error" << endl;
    }

}
