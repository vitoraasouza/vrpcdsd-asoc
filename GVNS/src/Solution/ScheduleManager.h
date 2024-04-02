#ifndef SCHEDULE_MANAGER_H_
#define SCHEDULE_MANAGER_H_

#include <vector>
#include <utility>
#include "Schedule.h"
#include "../Persistent/Instance.h"

struct ScheduleManager {
    Schedules schedules_;
    vector<bool> do_cd_operation_; // {1,...,|R|}
    vector<pair<int, int> > pic_del_; // {1,...,|R|}
    int num_requests_, num_vehicles_;

    ScheduleManager();
    ScheduleManager(const int num_requests, const int num_vehicles);

    void InsertUnloadReload(const int req, const int unload_pos, const int orig_vhc, const int dest_vhc);

    ScheduleManager & operator=(const ScheduleManager & schedule_manager);
    friend ostream & operator<<(ostream & out, const ScheduleManager & schedule_manager);

};

#endif // SCHEDULE_MANAGER_H_
