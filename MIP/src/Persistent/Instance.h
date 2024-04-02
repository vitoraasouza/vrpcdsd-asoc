#ifndef INSTANCE_H_
#define INSTANCE_H_

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <vector>

#include "Node.h"
#include "Vehicle.h"

typedef vector<vector<double> > AdjacencyMatrix;
typedef vector<int> ProcessingTimes;

class Instance {
    public:
        ~Instance();
        static void Init(char* instance_file_name);

        // Returns a pointer to the instance object. If it hasn't been initialized,
        // exits with an error message
        static Instance* instance();
        void ReadInstance(char* instance_file_name);
        void ProcessInstance();
        void PrintInstance();

        //getters
        char * name();
        int num_nodes() const;
        int num_requests() const;
        int num_suppliers() const;
        int num_customers() const;
        int num_vehicles() const;
        int unload_preparation_time() const;
        int reload_preparation_time() const;
        int between_adjacent_docks_time() const;
        int between_docks_time(int k1, int k2) const;
        const Nodes & nodes() const;
        const Node & node(const int node_num) const;
        const AdjacencyMatrix & adjacency_matrix() const;
        const ProcessingTimes & unloading_times() const;
        const ProcessingTimes & reloading_times() const;
        const Vehicle & vehicle() const;
        double EdgeCost(const int u, const int v) const;
        int unloading_time(const int i) const;
        int reloading_time(const int i) const;


    private:
        Instance();
        static Instance* instance_;

        char name_[100];
        int num_nodes_, num_requests_, num_suppliers_, num_customers_, num_vehicles_;
        int unload_preparation_time_, reload_preparation_time_, unloading_time_per_pallet_, reloading_time_per_pallet_, between_adjacent_docks_time_;
        Nodes nodes_;
        AdjacencyMatrix adjacency_matrix_;
        ProcessingTimes unloading_times_, reloading_times_;
        Vehicle vehicle_;
};

#endif /* INSTANCE_H_ */
