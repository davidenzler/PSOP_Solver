#include "localPool.hpp"

LocalPool::LocalPool() {
    threadId = -1;
    localPool = vector<deque<node>>();
}
void LocalPool::removeCurrentDepth() {
    // removes current depth's ready list when finished
    poolLock.lock();
    localPool.pop_back();
    poolLock.unlock();
}

void LocalPool::addCurrentDepth() {
    // adds a new ready_list for current depth.
    poolLock.lock();
    localPool.push_back(deque<node>());
    poolLock.unlock();
}

void LocalPool::push_back(node readyNode) {
    poolLock.lock();
    localPool.back().push_back(readyNode);
    poolLock.unlock();
}

node LocalPool::pop_back() {
    node removedNode;
    poolLock.lock();
    removedNode = localPool.back().back();
    localPool.back().pop_back();
    poolLock.unlock();

    return removedNode;
}

node LocalPool::back() {
    return localPool.back().back();
}

bool LocalPool::empty() {
    return localPool.back().empty();
}

node LocalPool::requestShallowNode(int searchDepth) {
    node removedNode;

    poolLock.lock();
    if(searchDepth < localPool.size() && !localPool.empty()) {
        if( !(localPool[searchDepth].empty()) ) {
            removedNode = localPool[searchDepth].back();
            localPool[searchDepth].pop_back();
        }
    }
    poolLock.unlock();
    
    return removedNode;
}

void LocalPool::setThreadId(int id) {
    threadId = id;
}

void LocalPool::clear() {
    poolLock.lock();
    localPool.clear();
    localPool.push_back(deque<node>());
    poolLock.unlock();
}

std::deque<node>::iterator LocalPool::erase(std::deque<node>::iterator target) {
    localPool.back().erase(target);
}

std::deque<node>::iterator LocalPool::begin() {
    return localPool.back().begin();
}

std::deque<node>::iterator LocalPool::end() {
    return localPool.back().begin();
}

bool operator<(const node& p1, const node& p2)
{
 
    // this will return true when second person
    // has greater height. Suppose we have p1.height=5
    // and p2.height=5.5 then the object which
    // have max height will be at the top(or
    // max priority)
    return p1.lb < p2.lb;
}
