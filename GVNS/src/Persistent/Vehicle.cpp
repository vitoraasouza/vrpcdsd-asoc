#include "Vehicle.h"

Vehicle::Vehicle() {
    capacity_ = -1;
}

int Vehicle::capacity() const {
    return capacity_;
}

void Vehicle::SetCapacity(const int capacity) {
    capacity_ = capacity;
}
