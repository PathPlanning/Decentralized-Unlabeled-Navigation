#ifndef CTP_SWAP_DIJKSTRA_HPP
#define CTP_SWAP_DIJKSTRA_HPP

#include <memory>
#include <unordered_map>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <xtensor-blas/xlinalg.hpp>


#include "node.hpp"
#include "visibility_graph.hpp"
#include "search_tree.hpp"


class Dijkstra {

public:
    Dijkstra() = default;

    explicit Dijkstra(const std::shared_ptr<VisibilityGraph> &graph) : graph(graph) {};
    std::unordered_map<int64_t , std::shared_ptr<Node<int64_t>>, std::hash<int64_t>>
    findPaths(const int64_t &goal) const;

protected:

    [[nodiscard]] double
    computeCost(const int64_t &v1_id, const int64_t &v2_id) const;

    [[nodiscard]] double
    computeDist(const int64_t &v1_id, const int64_t &v2_id) const;


    std::shared_ptr<VisibilityGraph> graph;


};


#endif //CTP_SWAP_DIJKSTRA_HPP
