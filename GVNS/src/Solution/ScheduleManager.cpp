#include "ScheduleManager.h"

ScheduleManager::ScheduleManager() {
    num_requests_ = 0;
    num_vehicles_ = 0;
}

ScheduleManager::ScheduleManager(const int num_requests, const int num_vehicles) {
    schedules_.resize(num_vehicles);
    do_cd_operation_.resize(num_requests + 1);
    pic_del_.resize(num_requests + 1);
    num_requests_ = num_requests;
    num_vehicles_ = num_vehicles;
}

void ScheduleManager::InsertUnloadReload(const int req, const int unload_pos, const int orig_vhc, const int dest_vhc) {
    UnloadUnit unload_unit;
    unload_unit.ind_ = req;

    // insert req on orig_vhc
    if (unload_pos == 0) {
        unload_unit.start_time_ = schedules_[orig_vhc].start_time_ + Instance::instance()->unload_preparation_time();
    } else {
        unload_unit.start_time_ = schedules_[orig_vhc].unloaded_reqs_[unload_pos - 1].start_time_ + Instance::instance()->unloading_time(schedules_[orig_vhc].unloaded_reqs_[unload_pos - 1].ind_);
    }

    for (uint i = unload_pos; i < schedules_[orig_vhc].unloaded_reqs_.size(); ++i) {
        schedules_[orig_vhc].unloaded_reqs_[i].start_time_ += Instance::instance()->unloading_time(unload_unit.ind_);
    }

    schedules_[orig_vhc].unloaded_reqs_.insert(schedules_[orig_vhc].unloaded_reqs_.begin() + unload_pos, unload_unit);
    schedules_[orig_vhc].unload_end_ = schedules_[orig_vhc].unloaded_reqs_[schedules_[orig_vhc].unloaded_reqs_.size() - 1].start_time_ + Instance::instance()->unloading_time(unload_unit.ind_);

    // update reload reqs on orig_vhc
    if (schedules_[orig_vhc].reloaded_reqs_.size() > 0) {
        double current_time = schedules_[orig_vhc].unload_end_ + Instance::instance()->reload_preparation_time();
        for (uint i = 0; i < schedules_[orig_vhc].reloaded_reqs_.size(); ++i) {
            if (cmp(current_time, schedules_[orig_vhc].reloaded_reqs_[i].start_time_) > 0) {
                schedules_[orig_vhc].reloaded_reqs_[i].start_time_ = current_time;
                current_time += Instance::instance()->reloading_time(schedules_[orig_vhc].reloaded_reqs_[i].ind_);
            } else {
                break;
            }
        }
        schedules_[orig_vhc].completion_time_ = schedules_[orig_vhc].reloaded_reqs_[schedules_[orig_vhc].reloaded_reqs_.size() - 1].start_time_
                                                + Instance::instance()->reloading_time(schedules_[orig_vhc].reloaded_reqs_[schedules_[orig_vhc].reloaded_reqs_.size() - 1].ind_);
    } else {
        schedules_[orig_vhc].completion_time_ = schedules_[orig_vhc].unload_end_;
    }

    // update reload reqs for moved unloaded reqs
    int current_req, current_dest_vhc;
    for (uint i = unload_pos + 1; i < schedules_[orig_vhc].unloaded_reqs_.size(); ++i) {
        current_req = schedules_[orig_vhc].unloaded_reqs_[i].ind_;
        current_dest_vhc = pic_del_[current_req].second;

        uint curr_rel_pos;
        for (curr_rel_pos = 0; curr_rel_pos < schedules_[current_dest_vhc].reloaded_reqs_.size(); ++curr_rel_pos) {
            if (schedules_[current_dest_vhc].reloaded_reqs_[curr_rel_pos].ind_ == current_req) {
                schedules_[current_dest_vhc].reloaded_reqs_[curr_rel_pos].availability_time_ += Instance::instance()->unloading_time(unload_unit.ind_);
                break;
            }
        }

        for (uint j = curr_rel_pos; j < schedules_[current_dest_vhc].reloaded_reqs_.size() - 1; ++j) {
            if (schedules_[current_dest_vhc].reloaded_reqs_[j+1] < schedules_[current_dest_vhc].reloaded_reqs_[j]) {
                ReloadUnit ru_aux = schedules_[current_dest_vhc].reloaded_reqs_[j];
                schedules_[current_dest_vhc].reloaded_reqs_[j] = schedules_[current_dest_vhc].reloaded_reqs_[j+1];
                schedules_[current_dest_vhc].reloaded_reqs_[j+1] = ru_aux;
            }
        }

        for (uint j = curr_rel_pos; j < schedules_[current_dest_vhc].reloaded_reqs_.size(); ++j) {
            if (j == 0) {
                schedules_[current_dest_vhc].reloaded_reqs_[j].start_time_ = max(schedules_[current_dest_vhc].reloaded_reqs_[j].availability_time_,
                                                                                 schedules_[current_dest_vhc].unload_end_ + Instance::instance()->reload_preparation_time());
            } else {
                schedules_[current_dest_vhc].reloaded_reqs_[j].start_time_ = max(schedules_[current_dest_vhc].reloaded_reqs_[j].availability_time_,
                                                                                 schedules_[current_dest_vhc].reloaded_reqs_[j-1].start_time_
                                                                                 + Instance::instance()->reloading_time(schedules_[current_dest_vhc].reloaded_reqs_[j-1].ind_));
            }
        }

        schedules_[current_dest_vhc].completion_time_ = schedules_[current_dest_vhc].reloaded_reqs_[schedules_[current_dest_vhc].reloaded_reqs_.size() - 1].start_time_
                                                        + Instance::instance()->reloading_time(schedules_[current_dest_vhc].reloaded_reqs_[schedules_[current_dest_vhc].reloaded_reqs_.size() - 1].ind_);
    }

    // insert req on dest_vhc
    ReloadUnit reload_unit;
    reload_unit.ind_ = req;
    reload_unit.availability_time_ = unload_unit.start_time_
                                     + Instance::instance()->unloading_time(unload_unit.ind_)
                                     + Instance::instance()->between_docks_time(orig_vhc, dest_vhc);

    uint insertion_pos;
    if (schedules_[dest_vhc].reloaded_reqs_.size() == 0) {
        schedules_[dest_vhc].reloaded_reqs_.push_back(reload_unit);
        insertion_pos = 0;
    } else {
        for (insertion_pos = 0; insertion_pos < schedules_[dest_vhc].reloaded_reqs_.size(); ++insertion_pos) {
            if (reload_unit < schedules_[dest_vhc].reloaded_reqs_[insertion_pos]) {
                schedules_[dest_vhc].reloaded_reqs_.insert(schedules_[dest_vhc].reloaded_reqs_.begin() + insertion_pos, reload_unit);
                break;
            }
        }
        if (insertion_pos == schedules_[dest_vhc].reloaded_reqs_.size()) {
            schedules_[dest_vhc].reloaded_reqs_.insert(schedules_[dest_vhc].reloaded_reqs_.begin() + insertion_pos, reload_unit);
        }
    }

    for (uint j = insertion_pos; j < schedules_[dest_vhc].reloaded_reqs_.size(); ++j) {
        if (j == 0) {
            schedules_[dest_vhc].reloaded_reqs_[j].start_time_ = max(schedules_[dest_vhc].reloaded_reqs_[j].availability_time_,
                                                                     schedules_[dest_vhc].unload_end_ + Instance::instance()->reload_preparation_time());
        } else {
            schedules_[dest_vhc].reloaded_reqs_[j].start_time_ = max(schedules_[dest_vhc].reloaded_reqs_[j].availability_time_,
                                                                     schedules_[dest_vhc].reloaded_reqs_[j-1].start_time_
                                                                     + Instance::instance()->reloading_time(schedules_[dest_vhc].reloaded_reqs_[j-1].ind_));
        }
    }

    schedules_[dest_vhc].completion_time_ = schedules_[dest_vhc].reloaded_reqs_[schedules_[dest_vhc].reloaded_reqs_.size() - 1].start_time_
                                            + Instance::instance()->reloading_time(schedules_[dest_vhc].reloaded_reqs_[schedules_[dest_vhc].reloaded_reqs_.size() - 1].ind_);

    // update ScheduleManager structures
    do_cd_operation_[req] = true;
    pic_del_[req] = make_pair(orig_vhc, dest_vhc);
}

ScheduleManager & ScheduleManager::operator=(const ScheduleManager & schedule_manager) {
    schedules_ = schedule_manager.schedules_;
    pic_del_ = schedule_manager.pic_del_;
    do_cd_operation_ = schedule_manager.do_cd_operation_;
    num_requests_ = schedule_manager.num_requests_;
    num_vehicles_ = schedule_manager.num_vehicles_;

    return * this;
}

ostream & operator<<(ostream & out, const ScheduleManager & schedule_manager) {
    out << schedule_manager.schedules_;

    return out;
}
