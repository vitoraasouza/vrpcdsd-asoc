#include "Node.h"

Node::Node() {
    x_ = -1.0;
    y_ = -1.0;
    id_ = -1;
    demand_ = -1;
}

Node::Node(const double x, const double y, const int id, const int demand) {
    x_ = x;
    y_ = y;
    id_ = id;
    demand_ = demand;
}

double Node::x() const {
    return x_;
}

double Node::y() const {
    return y_;
}

int Node::id() const {
    return id_;
}

int Node::demand() const {
    return demand_;
}

void Node::SetX(const double x) {
    x_ = x;
}

void Node::SetY(const double y) {
    y_ = y;
}

void Node::SetId(const int id) {
    id_ = id;
}

void Node::SetDemand(const int demand) {
    demand_ = demand;
}
