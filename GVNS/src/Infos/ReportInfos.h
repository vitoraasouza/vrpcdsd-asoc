#ifndef REPORT_INFOS_H_
#define REPORT_INFOS_H_

struct ReportInfos {
    double overall_time_;
    double overall_obj_value_;
    ReportInfos() {
        overall_time_ = 0.0;
        overall_obj_value_ = -1.0;
    }
};

#endif // REPORT_INFOS_H_
