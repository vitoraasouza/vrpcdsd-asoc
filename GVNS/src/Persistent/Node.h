#ifndef NODE_H_
#define NODE_H_

#include <vector>
using namespace std;

class Node {
    public:
        Node();
        Node(double x, double y, int id, int demand);

        // getters
        double x() const;
        double y() const;
        int id() const;
        int demand() const;

        //setters
        void SetX(const double x);
        void SetY(const double y);
        void SetId(const int id);
        void SetDemand(const int demand);

    private:
        double x_, y_;
        int id_, demand_;
};

typedef vector<Node> Nodes;

#endif /* NODE_H_ */
