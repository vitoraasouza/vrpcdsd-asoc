#ifndef COMPACT_MODEL_H_
#define COMPACT_MODEL_H_

#include <cmath>
#include <set>
#include <sstream>

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

#include "../Infos/InputSettings.h"
#include "../Infos/ReportInfos.h"
#include "../Persistent/Instance.h"
#include "../Utils/Utils.h"

using namespace std;

struct CompactModel {

    int num_vehicles_, num_real_nodes_, num_model_nodes_, num_suppliers_, num_customers_, num_requests_, num_edges_;
    int first_supplier_, last_supplier_, first_customer_, last_customer_, orig_, dest_;
    int bigM1_, bigM2_, bigM3_, bigM4_;
    InputSettings input_settings_;
    ReportInfos report_infos_;

    vector<vector<int> > x_index_;
    vector<vector<int> > z_index_;

    // ambiente
    IloEnv env_;

    // modelo
    IloModel master_;

    // ambiente, modelo e variaveis
    IloCplex * master_cpx_;

    // variaveis
    IloArray<IloIntVarArray> lambda_; // x_[k][x_index_[i][j]]
    IloArray<IloIntVarArray> y_; // y_[k][i]
    IloArray<IloArray<IloNumVarArray> > f_; // f_[k][i][j]
    IloArray<IloNumVarArray> s_; // su_[k][i]
    IloArray<IloNumVarArray> sr_; // sr_[k][i]
    IloArray<IloIntVarArray> z_; // zu_[k][zur_index_[i][j]]
    IloArray<IloIntVarArray> zr_; // zr_[k][zur_index_[i][j]]
    IloNumVarArray completion_; // D_[k]
    IloNumVarArray C_; // C_[k]
    IloArray<IloIntVarArray> lu_; // lu_[k][i]
    IloArray<IloIntVarArray> lr_; // lr_[k][i]
    IloIntVarArray u_; // lu_[k]
    IloIntVarArray r_; // lr_[k]

    IloNumVar Cmax_;

    // funcao objetivo
    IloObjective objective_;

    // restricoes
    IloRangeArray constraints_2_;
    IloRangeArray constraints_3_;
    IloRangeArray constraints_4_;
    IloRangeArray constraints_5_;
    IloRangeArray constraints_6_;
    IloRangeArray constraints_7_;
    IloRangeArray constraints_8_;
    IloRangeArray constraints_9_;
    IloRangeArray constraints_10_;
    IloRangeArray constraints_11_;
    IloRangeArray constraints_12_;
    IloRangeArray constraints_13_;
    IloRangeArray constraints_14_;
    IloRangeArray constraints_15_;
    IloRangeArray constraints_16_;
    IloRangeArray constraints_17_;
    IloRangeArray constraints_18_;
    IloRangeArray constraints_19_;
    IloRangeArray constraints_20_;
    IloRangeArray constraints_21_;
    IloRangeArray constraints_22_;
    IloRangeArray constraints_23_;
    IloRangeArray constraints_24_;
    IloRangeArray constraints_25_;
    IloRangeArray constraints_26_;
    IloRangeArray constraints_27_;
    IloRangeArray constraints_28_;
    IloRangeArray constraints_29_;
    IloRangeArray constraints_30_;
    IloRangeArray constraints_31_;
    IloRangeArray constraints_32_;
    IloRangeArray constraints_33_;
    IloRangeArray constraints_34_;
    IloRangeArray constraints_35_;
    IloRangeArray constraints_36_;
    IloRangeArray constraints_37_;
    IloRangeArray constraints_38_;
    IloRangeArray constraints_Cmax_;


    CompactModel(InputSettings input_settings);

    // desativa os parametros default do cplex (presolve, cuts...)
    void turn_off_parameters();

    // cria modelo
    void create_model();

    // cria variaveis
    void create_variables();

    // cria funcao objetivo
    void create_objective_function();

    // calcula valores pequenos para os big-Ms
    void calculate_big_Ms();

    // cria restricoes
    void create_constraints();

    void create_routing_constraints();
    void create_delivery_routing_constraints();
    void create_CD_constraints();
    void create_reload_at_CD_constraints();
    void create_makespan_constraints();

    void printSolutionOnFile(const char * solution_file_name);
    void printTikZSolutionGuide(const char * tikz_guide_file_name);
    void printTikZRelaxationGuide(const char * tikz_relaxation_file_name);
    void printReport();

    void solve();

};

#endif // COMPACT_MODEL_H_
