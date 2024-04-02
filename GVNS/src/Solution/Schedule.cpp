#include "Schedule.h"

Schedule::Schedule() {
    completion_time_ = -1.0;
    unload_end_ = -1.0;
}

Schedule & Schedule::operator=(const Schedule & schedule) {
    start_time_ = schedule.start_time_;
    unload_end_ = schedule.unload_end_;
    completion_time_ = schedule.completion_time_;
    unloaded_reqs_ = schedule.unloaded_reqs_;
    reloaded_reqs_ = schedule.reloaded_reqs_;

    return *this;
}

ostream & operator<<(ostream & out, const Schedule & schedule) {
    for (uint i = 0; i < schedule.unloaded_reqs_.size(); ++i) {
        out << "U" << schedule.unloaded_reqs_[i].ind_ << "(" << schedule.unloaded_reqs_[i].start_time_ << ") ";
    }
    out << "  ---   ";
    for (uint i = 0; i < schedule.reloaded_reqs_.size(); ++i) {
        out << "R" << schedule.reloaded_reqs_[i].ind_ << "(" << schedule.reloaded_reqs_[i].start_time_ << ") ";
    }
    out << "  -   st=" << schedule.start_time_ << " / ue="<< schedule.unload_end_ << " / ct=" << schedule.completion_time_;

    return out;
}

ostream & operator<<(ostream & out, const Schedules & schedules) {
    double total_completion_time = 0;
    for (uint i = 0; i < schedules.size(); ++i) {
        out << "k" << i+1 << ": " << schedules[i] << endl;
        total_completion_time += schedules[i].completion_time_;
    }
    out << "SUM = " << total_completion_time << endl;

    return out;
}
