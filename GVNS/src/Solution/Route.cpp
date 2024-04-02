#include "Route.h"

Route::Route() {
    visited_nodes_.push_back(0);
    visited_nodes_.push_back(0);
    load_ = 0;
    num_nodes_ = 0;
    cost_ = 0.0;
    last_insertable_pos_ = first_insertable_pos_;
}

double Route::SimulateVisitedNodeInsertion(const int pos, const int ind) {
    double simulated_cost = cost_;
    simulated_cost -= Instance::instance()->EdgeCost(visited_nodes_[pos-1], visited_nodes_[pos]);
    simulated_cost += Instance::instance()->EdgeCost(visited_nodes_[pos-1], ind) + Instance::instance()->EdgeCost(ind, visited_nodes_[pos]);
    return simulated_cost;
}

void Route::InsertVisitedNode(const int pos, const int ind) {
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[pos-1], visited_nodes_[pos]);
    cost_ += Instance::instance()->EdgeCost(visited_nodes_[pos-1], ind) + Instance::instance()->EdgeCost(ind, visited_nodes_[pos]);
    visited_nodes_.insert(visited_nodes_.begin() + pos, ind);
    load_ += Instance::instance()->node(ind).demand();
    ++last_insertable_pos_;
    ++num_nodes_;
}

void Route::RemoveVisitedNodeFromPos(const int pos) {
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[pos-1], visited_nodes_[pos]);
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[pos], visited_nodes_[pos+1]);
    cost_ += Instance::instance()->EdgeCost(visited_nodes_[pos-1], visited_nodes_[pos+1]);
    load_ -= Instance::instance()->node(visited_nodes_[pos]).demand();
    visited_nodes_.erase(visited_nodes_.begin() + pos);
    --last_insertable_pos_;
    --num_nodes_;
}

void Route::ReversePartVisitedNodes(const int first_pos, const int last_pos) {
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[first_pos-1], visited_nodes_[first_pos]);
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[last_pos], visited_nodes_[last_pos+1]);

    reverse(visited_nodes_.begin() + first_pos, visited_nodes_.begin() + (last_pos + 1));

    cost_ += Instance::instance()->EdgeCost(visited_nodes_[first_pos-1], visited_nodes_[first_pos]);
    cost_ += Instance::instance()->EdgeCost(visited_nodes_[last_pos], visited_nodes_[last_pos+1]);
}

void Route::SwapVisitedNodes(const int pos_a, const int pos_b) {
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[pos_a-1], visited_nodes_[pos_a]);
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[pos_a], visited_nodes_[pos_a+1]);
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[pos_b-1], visited_nodes_[pos_b]);
    cost_ -= Instance::instance()->EdgeCost(visited_nodes_[pos_b], visited_nodes_[pos_b+1]);

    swap(visited_nodes_[pos_a], visited_nodes_[pos_b]);

    cost_ += Instance::instance()->EdgeCost(visited_nodes_[pos_a-1], visited_nodes_[pos_a]);
    cost_ += Instance::instance()->EdgeCost(visited_nodes_[pos_a], visited_nodes_[pos_a+1]);
    cost_ += Instance::instance()->EdgeCost(visited_nodes_[pos_b-1], visited_nodes_[pos_b]);
    cost_ += Instance::instance()->EdgeCost(visited_nodes_[pos_b], visited_nodes_[pos_b+1]);
}

void Route::Clear() {
    visited_nodes_ = vector<int>(2,0);
    load_ = 0;
    num_nodes_ = 0;
    cost_ = 0.0;
    last_insertable_pos_ = first_insertable_pos_;
}

Route & Route::operator=(const Route & route) {
    visited_nodes_ = route.visited_nodes_;
    load_ = route.load_;
    num_nodes_ = route.num_nodes_;
    cost_ = route.cost_;
    last_insertable_pos_ = route.last_insertable_pos_;

    return *this;
}

string Route::toString() const {
    char route_data[200];
    int num_requests = Instance::instance()->num_requests();
    sprintf(route_data, "( ");
    for (VisitedNodes::const_iterator it = visited_nodes_.begin(); it != visited_nodes_.end(); it++) {
        sprintf(route_data + strlen(route_data), "%d ", *it > num_requests ? *it - num_requests : *it);
    }
    sprintf(route_data + strlen(route_data), ") - load = %d - cost = %.3lf", load_ , cost_);
    return string(route_data);
}

ostream & operator<<(ostream & out, const Route & route) {
    out << route.toString();

    return out;
}

ostream & operator<<(ostream & out, const Routes & routes) {
    double sum_routes = 0.0;
    for (uint r = 0; r < routes.size(); ++r) {
        out << "k" << r+1 << ": " << routes[r] << endl;
        sum_routes += routes[r].cost_;
    }
    out << "SUM = " << sum_routes << endl;

    return out;
}
