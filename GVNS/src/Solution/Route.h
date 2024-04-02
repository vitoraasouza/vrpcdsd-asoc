#ifndef ROUTE_H_
#define ROUTE_H_

#include <string>
#include <algorithm>
#include <iostream>

#include "../Persistent/Instance.h"

using namespace std;

typedef vector<int> VisitedNodes;

struct Route {

    VisitedNodes visited_nodes_;
    int load_;
    int num_nodes_;
    double cost_;
    int last_insertable_pos_;

    static const int first_insertable_pos_ = 1;

    //constructor
    Route();

    //setters
    double SimulateVisitedNodeInsertion(const int pos, const int ind);
    void InsertVisitedNode(const int pos, const int ind);
    void RemoveVisitedNodeFromPos(const int pos);
    void ReversePartVisitedNodes(const int first_pos, const int last_pos);
    void SwapVisitedNodes(const int pos_a, const int pos_b);
    void Clear();

    //general methods
    string toString() const;
    Route & operator=(const Route & route);
    friend ostream & operator<<(ostream & out, const Route & route);

};

typedef vector<Route> Routes;
ostream & operator<<(ostream & out, const Routes & routes);

#endif /* ROUTE_H_ */
