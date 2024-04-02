#include "Edge.h"

Edge::Edge(const int u, const int v, const double value) {
    u_ = u;
    v_ = v;
    value_ = value;
}

int Edge::u() const {
    return u_;
}

int Edge::v() const {
    return v_;
}

double Edge::value() const {
    return value_;
}

bool Edge::operator<(const Edge & edge) const {
    return value_ < edge.value();
}
