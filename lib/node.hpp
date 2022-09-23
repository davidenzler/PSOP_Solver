#ifndef NODE_H
#define NODE_H

#include "history.hpp"
#include "active_tree.hpp"


class node {
    public:
        int n;
        int lb;
        int nc;
        int partial_cost;
        bool node_invalid;
        HistoryNode* his_entry;
        Active_Node* act_entry = NULL;
        bool pushed;
        node(int id, int lb);
        node();
        node(const node &src);
};

#endif