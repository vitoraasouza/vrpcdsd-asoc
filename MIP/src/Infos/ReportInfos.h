#ifndef REPORT_INFOS_H_
#define REPORT_INFOS_H_

#include <ilcplex/ilocplex.h>

struct ReportInfos {
    double root_time_, overall_time_;
    IloAlgorithm::Status root_status_, overall_status_;
    double root_dual_obj_value_, overall_dual_obj_value_, overall_int_obj_value_;
    int num_tree_nodes_, num_threads_;
    double gap_;
    ReportInfos() {
        root_time_ = overall_time_ = 0.0;
        root_dual_obj_value_ = overall_dual_obj_value_ = overall_int_obj_value_ = gap_ = -1.0;
        num_tree_nodes_ = num_threads_ = 0;
        root_status_ = overall_status_ = IloAlgorithm::Unknown;
    }
};

#endif // REPORT_INFOS_H_
