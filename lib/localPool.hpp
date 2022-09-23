#ifndef LOCALPOOL_H
#define LOCALPOOL_H 
#include "node.hpp"


/**
 * @brief a threadsafe vector of priority queues
 * entry 0 is the root of the search tree. Every entry is another consecutive depth in the tree
 * The dequeue contains nodes of depCnt 0
 * 
 */
class LocalPool {
    public:
        /**
         * @brief remove pool from recusrive stack when finished at tree depth
         * 
         */
        void removeCurrentDepth();

        /**
         * @brief add pool to recursive stack when starting new depth
         * 
         */
        void addCurrentDepth();

        /**
         * @brief add a node to the ready_list of current depth
         * 
         */
        void push_back(node);

        /**
         * @brief Set the Thread Id to match solver::threadid
         * 
         * @param id 
         */
        void setThreadId(int id);

        /**
         * @brief check if ready_list empty
         * 
         * @return true  ready_list for current depth empty
         * @return false ready_list for current depth has at least 1 elem
         */
        bool empty();

        /**
         * @brief pop a node from the readylist queue
         * 
         * @return node removed from ready_list queue
         */
        node pop_back();

        node back();

        /**
         * @brief will grab a random node from localPool.
         * Prioritiezes nodes located in shallower ready_lists
         * 
         * @param max_depth limits depth of search
         * @return node 
         */
        node requestShallowNode(int max_depth);
        
        /**
         * @brief resets the localPool to default empty state
         * 
         */
        void clear();

        std::deque<node>::iterator erase(std::deque<node>::iterator target);
        std::deque<node>::iterator begin();
        std::deque<node>::iterator end();

        /**
         * @brief Construct a new Local Pool object
         * 
         */
        LocalPool();


        size_t size() {
            return localPool.back().size();
        }

        void sort();

    private:
        vector<deque<node>> localPool;
        int threadId;
        mutable mutex poolLock;
        vector<bool> sorted;
};

#endif