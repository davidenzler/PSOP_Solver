#include "node.hpp"


node::node() {
    n = -1;
    lb = -1;
    nc = -1;
    partial_cost = -1;
    node_invalid = false;
    his_entry = NULL;
    act_entry = NULL;
    pushed = false;
}

node::node(const node &src) {
    n = src.n;
    lb = src.lb;
    nc = src.nc;
    partial_cost = src.partial_cost;
    node_invalid = src.node_invalid;
    his_entry = src.his_entry;
    act_entry = src.act_entry;
    pushed = src.pushed;
}

node::node(int id, int _lb) {
    n = id;
    lb = _lb;
    nc = -1;
    partial_cost = -1;
    node_invalid = false;
    his_entry = NULL;
    act_entry = NULL;
    pushed = false;
}