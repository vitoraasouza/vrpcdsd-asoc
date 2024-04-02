#ifndef EDGE_H_
#define EDGE_H_

class Edge {
    public:
        Edge(const int u, const int v, const double value);

        //getters
        int u() const;
        int v() const;
        double value() const;

        bool operator<(const Edge & edge) const;

    private:
        int u_, v_;
        double value_;
};

#endif /* EDGE_H_ */
