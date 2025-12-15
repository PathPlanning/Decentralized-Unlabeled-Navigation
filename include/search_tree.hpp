#ifndef CTP_SWAP_SEARCHTREE_HPP
#define CTP_SWAP_SEARCHTREE_HPP

#include "xtensor/xtensor.hpp"
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <queue>
#include <memory>


#include "grid_map.hpp"
#include "node.hpp"

template<typename T, typename HashT>
class SearchTree {

public:
    SearchTree() = default;

    [[nodiscard]] bool openIsEmpty() const { return open.empty(); }

    void addToOpen(std::shared_ptr<Node<T>> item) { open.push(item); }

    std::shared_ptr<Node<T>> getBestFromOpen() {
        while (not openIsEmpty()) {
            std::shared_ptr<Node<T>> best_node = open.top();
            open.pop();
            if (not wasExpanded(best_node)) {
                return best_node;
            }
        }
        return nullptr;
    }

    void addToClosed(std::shared_ptr<Node<T>> item) { closed[item->node_id] = item; }

    bool wasExpanded(std::shared_ptr<Node<T>> item) const {

        return closed.find(item->node_id) != closed.end();
    }

    std::unordered_map<T, std::shared_ptr<Node<T>>, HashT> getExpanded() const {
        return closed;
    }


protected:
    std::unordered_map<T, std::shared_ptr<Node<T>>, HashT> closed;
    std::priority_queue<std::shared_ptr<Node<T>>, std::vector<std::shared_ptr<Node<T>>>, NodeCompare<T>> open;
};

#endif // CTP_SWAP_SEARCHTREE_HPP
