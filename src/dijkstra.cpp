#include "dijkstra.hpp"


std::unordered_map<int64_t, std::shared_ptr<Node<int64_t>>, std::hash<int64_t>> Dijkstra::findPaths(const int64_t &goal) const {
    auto ast = SearchTree<int64_t, std::hash<int64_t>>();
    auto curr_node = std::make_shared<Node<int64_t>>();
    curr_node->node_id = goal;
    curr_node->g = 0.0;
    curr_node->len = 0.0;
    curr_node->parent = nullptr;
    ast.addToOpen(curr_node);

    while (not ast.openIsEmpty()) {
        curr_node = ast.getBestFromOpen();

        if (curr_node == nullptr) {
            break;
        }
        ast.addToClosed(curr_node);
        auto neighbors = graph->findNeighbors(curr_node->node_id);
        for (const auto &neighbor: neighbors) {
            const auto new_node = std::make_shared<Node<int64_t>>();
            new_node->node_id = neighbor;
            if (not ast.wasExpanded(new_node)) {
                new_node->g = curr_node->g + computeCost(curr_node->node_id, new_node->node_id);
                new_node->len = curr_node->len + computeDist(curr_node->node_id, new_node->node_id);
                new_node->parent = curr_node;
                ast.addToOpen(new_node);
            }
        }
    }
    return ast.getExpanded();
}

double Dijkstra::computeCost(const int64_t &v1_id, const int64_t &v2_id) const {
    return computeDist(v1_id, v2_id);
}

double Dijkstra::computeDist(const int64_t &v1_id, const int64_t &v2_id) const {
    xt::xtensor<double, 1> p1, p2;
    p1 = graph->getPoint(v1_id);
    p2 = graph->getPoint(v2_id);
    return xt::linalg::norm(p2 - p1);
}
