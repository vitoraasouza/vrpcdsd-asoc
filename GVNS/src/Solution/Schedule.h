#ifndef SCHEDULE_H_
#define SCHEDULE_H_

#include <iostream>
#include <vector>
#include "../Utils/Utils.h"

using namespace std;

struct UnloadUnit {
    int ind_;
    double start_time_;
    UnloadUnit() {
        ind_ = 0;
        start_time_ = 0.0;
    }
    UnloadUnit(int ind) {
        ind_ = ind;
        start_time_ = 0.0;
    }
    UnloadUnit(const int ind, const double start_time) {
        ind_ = ind;
        start_time_ = start_time;
    }
};

struct ReloadUnit {
    int ind_;
    double availability_time_, start_time_;
    ReloadUnit() {
        ind_ = 0;
        availability_time_ = start_time_ = 0.0;
    }
    ReloadUnit(int ind) {
        ind_ = ind;
        availability_time_ = start_time_ = 0.0;
    }
    ReloadUnit(int ind, double availability_time) {
        ind_ = ind;
        availability_time_ = availability_time;
        start_time_ = 0.0;
    }
    friend bool operator<(const ReloadUnit & a, const ReloadUnit & b) {
        return cmp(a.availability_time_,  b.availability_time_) < 0;
    }
};

typedef vector<UnloadUnit> UnloadSchedule;
typedef vector<ReloadUnit> ReloadSchedule;

struct Schedule {
    UnloadSchedule unloaded_reqs_;
    ReloadSchedule reloaded_reqs_;
    double start_time_, unload_end_, completion_time_;

    Schedule();

    Schedule & operator=(const Schedule & schedule);
    friend ostream & operator<<(ostream & out, const Schedule & schedule);

};

typedef vector<Schedule> Schedules;
ostream & operator<<(ostream & out, const Schedules & schedules);

#endif // SCHEDULE_H_
