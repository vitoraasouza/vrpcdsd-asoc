#ifndef VEHICLE_H_
#define VEHICLE_H_

class Vehicle {
    public:
        Vehicle();

        //getters
        int capacity() const;

        //setters
        void SetCapacity(const int capacity);

    private:
        int capacity_;
};

#endif /* VEHICLE_H_ */
