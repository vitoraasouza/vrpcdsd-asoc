#include "Instance.h"

Instance* Instance::instance_ = NULL;

void Instance::Init(char* instance_file_name) {

    instance_ = new Instance();

    // Read dada files.
    instance_->ReadInstance(instance_file_name);
    instance_->ProcessInstance();

}

Instance* Instance::instance() {
//    if (instance_ == NULL) {
//        printf("Error - Instance not initialized!\n");
//        exit(1);
//    }
    return instance_;
}

Instance::Instance() {

}

char * Instance::name() {
    return name_;
}

int Instance::num_nodes() const {
    return num_nodes_;
}

int Instance::num_requests() const {
    return num_requests_;
}

int Instance::num_suppliers() const {
    return num_suppliers_;
}

int Instance::num_customers() const {
    return num_customers_;
}

int Instance::num_vehicles() const {
    return num_vehicles_;
}

int Instance::unload_preparation_time() const {
    return unload_preparation_time_;
}

int Instance::reload_preparation_time() const {
    return reload_preparation_time_;
}

int Instance::between_adjacent_docks_time() const {
    return between_adjacent_docks_time_;
}

int Instance::between_docks_time(int k1, int k2) const {
    return between_adjacent_docks_time_ * abs(k2 - k1);
}

const Nodes & Instance::nodes() const {
    return nodes_;
}

const Node & Instance::node(const int node_num) const {
    return nodes_[node_num];
}

const AdjacencyMatrix & Instance::adjacency_matrix() const {
    return adjacency_matrix_;
}

const ProcessingTimes & Instance::unloading_times() const {
    return unloading_times_;
}

const ProcessingTimes & Instance::reloading_times() const {
    return reloading_times_;
}

const Vehicle & Instance::vehicle() const {
    return vehicle_;
}

double Instance::EdgeCost(const int u, const int v) const {
    return adjacency_matrix_[u][v];
}

int Instance::unloading_time(const int i) const {
    return unloading_times_[i];
}

int Instance::reloading_time(const int i) const {
    return reloading_times_[i];
}

void Instance::ReadInstance(char* instance_file_name_and_dir) {
    ifstream instance_file;
    instance_file.open(instance_file_name_and_dir);

    int id, demand, capacity;
    double x, y;
    string trash;

    char * begin_file_name = 1 + strrchr(instance_file_name_and_dir, '/');
    if (begin_file_name == NULL) {
        begin_file_name = instance_file_name_and_dir;
    }
    char * end_file_name = strrchr(instance_file_name_and_dir, '.');
    strncpy(name_, begin_file_name, end_file_name - begin_file_name);
    name_[end_file_name - begin_file_name] = '\0';

    instance_file >> trash;
    instance_file >> trash;

    instance_file >> num_requests_;
    num_nodes_ = 2 * num_requests_ + 1;
    num_suppliers_ = num_customers_ = num_requests_;

    instance_file >> trash;
    instance_file >> trash >> trash;

    instance_file >> num_vehicles_ >> capacity;
    vehicle_.SetCapacity(capacity);

    for (int i = 0; i < 9; ++i) {
        instance_file >> trash;
    }

    nodes_.resize(num_nodes_);

    instance_file >> id >> x >> y;
    instance_file >> trash >> trash >> trash;
    nodes_[0] = Node(x, y, id, 0);

    double xs, ys, xc, yc;
    for (int i = 0; i < num_requests_; ++i) {
        instance_file >> id >> xs >> ys >> xc >> yc >> demand;
        nodes_[id] = Node(xs, ys, id, demand);
        nodes_[id + num_requests_] = Node(xc, yc, id + num_requests_, demand);
    }

    instance_file >> trash;
    instance_file >> trash >> trash >> trash;

    int preparation_time, time_per_pallet;
    instance_file >> preparation_time >> time_per_pallet >> between_adjacent_docks_time_;
    unload_preparation_time_ = reload_preparation_time_ = preparation_time;
    unloading_time_per_pallet_ = reloading_time_per_pallet_ = time_per_pallet;
}

void Instance::ProcessInstance() {
    // Calcula a distancia entre todos os vertices do grafo e armazena numa matriz de adj.
    adjacency_matrix_.resize(num_nodes_);
    for (int i = 0; i < num_nodes_; ++i) {
        adjacency_matrix_[i].resize(num_nodes_);
        for (int j = 0; j < num_nodes_; ++j) {
            adjacency_matrix_[i][j] = sqrt(pow(nodes_[i].x() - nodes_[j].x(), 2.0) + pow(nodes_[i].y() - nodes_[j].y(), 2.0));
        }
    }

    unloading_times_.resize(num_requests_ + 1);
    reloading_times_.resize(num_requests_ + 1);
    for (int i = 1; i <= num_requests_; ++i) {
        unloading_times_[i] = nodes_[i].demand() * unloading_time_per_pallet_;
        reloading_times_[i] = nodes_[i].demand() * reloading_time_per_pallet_;
    }
}

void Instance::PrintInstance() {

    printf("INSTANCE_NAME: %s\n\n", name_);
    printf("%s %18s %18s\n", "NUM_REQUESTS", "NUM_VEHICLES", "VEHICLE_CAP.");
    printf("%7d %18d %18d\n\n", num_requests_, num_vehicles_, vehicle_.capacity());
    printf("%s %18s %18s\n", "   NUM_NODES", "NUM_SUPPLIERS", "NUM_CUSTOMERS");
    printf("%9d %16d %18d\n\n", num_nodes_, num_suppliers_, num_customers_);
    printf("%s %18s\n", "UN.PREP.TIME", "RE.PREP.TIME");
    printf("%7d %18d\n\n", unload_preparation_time_, reload_preparation_time_);

    printf("----------------------------------------------------------------------------\n");
    printf("       %19s %19s\n", "SUPPLIERS", "CUSTOMERS");
    printf("%5s| %9s %9s %9s %9s %9s %9s %9s\n", "#node", "x", "y", "x", "y", "demand", "un.time", "re.time");
    printf("----------------------------------------------------------------------------\n");
    printf(" %-4d| %9.1lf %9.1lf %9s %9s %9s %9s %9s\n", nodes_[0].id(), nodes_[0].x(), nodes_[0].y(), "-", "-", "-", "-", "-");
    for (int i = 1; i <= num_requests_; ++i) {
        printf(" %-4d| %9.1lf %9.1lf %9.1lf %9.1lf %9d %9d %9d\n", nodes_[i].id(), nodes_[i].x(), nodes_[i].y(), nodes_[i+num_requests_].x(), nodes_[i+num_requests_].y(), nodes_[i].demand(), unloading_times_[i], reloading_times_[i]);
    }
    printf("----------------------------------------------------------------------------\n");

    printf("\n");
    printf("------"); for (int i = 0; i <= num_requests_; ++i) { printf("---------"); } printf("\n");
    printf(" %s\n", "SUPPLIERS ADJ. MATRIX");
    printf("      "); for (int i = 0; i <= num_requests_; ++i) { printf(" %8d", i); } printf("\n");
    printf("------"); for (int i = 0; i <= num_requests_; ++i) { printf("---------"); } printf("\n");
    for (int i = 0; i <= num_suppliers_; ++i) {
        printf(" %-4d|", i);
        for (int j = 0; j <= i; ++j) {
            if (i == j) {
                printf(" %8s", "-");
            } else {
                printf(" %8.1lf", adjacency_matrix_[i][j]);
            }
        }
        printf("\n");
    }
    printf("------"); for (int i = 0; i <= num_requests_; ++i) { printf("---------"); } printf("\n");

    printf("\n");
    printf("------"); for (int i = 0; i <= num_requests_; ++i) { printf("---------"); } printf("\n");
    printf(" %s\n", "CUSTOMERS ADJ. MATRIX");
    printf("      "); for (int i = 0; i <= num_requests_; ++i) { printf(" %8d", i); } printf("\n");
    printf("------"); for (int i = 0; i <= num_requests_; ++i) { printf("---------"); } printf("\n");
    for (int i = 0; i <= num_requests_; ++i) {
        printf(" %-4d|", i);
        for (int j = 0; j <= i; ++j) {
            if (i == j) {
                printf(" %8s", "-");
            } else {
                printf(" %8.1lf", adjacency_matrix_[i > 0 ? i + num_requests_: i][j > 0 ? j + num_requests_ : j]);
            }
        }
        printf("\n");
    }
    printf("------"); for (int i = 0; i <= num_requests_; ++i) { printf("---------"); } printf("\n");
}
