#ifndef CTP_SWAP_THETA_STAR_HPP
#define CTP_SWAP_THETA_STAR_HPP

#include "xtensor/xtensor.hpp"
#include <cstdint>
#include <memory>
#include <unordered_set>
#include <utility>

#include "grid_map.hpp"
#include "node.hpp"
#include "search_tree.hpp"

class ThetaStar {

public:
    ThetaStar() = default;

    explicit ThetaStar(std::shared_ptr<GridMap> grid_map) : grid_map(std::move(grid_map)) {};

    std::unordered_map<std::pair<int64_t, int64_t>, std::shared_ptr<Node<std::pair<int64_t, int64_t>>>, GridPosHash>
    findPaths(const std::pair<int64_t, int64_t> &goal);

protected:

    [[nodiscard]] double
    computeCost(const std::pair<int64_t, int64_t> &ij_pos1, const std::pair<int64_t, int64_t> &ij_pos2) const;

    [[nodiscard]] double
    computeDist(const std::pair<int64_t, int64_t> &ij_pos1, const std::pair<int64_t, int64_t> &ij_pos2) const;

    std::shared_ptr<Node<std::pair<int64_t, int64_t>>>
    resetParent(std::shared_ptr<Node<std::pair<int64_t, int64_t>>> current,
                const std::shared_ptr<Node<std::pair<int64_t, int64_t>>> &parent);

    bool lineOfSight(const std::pair<int64_t, int64_t> &ij_pos1, const std::pair<int64_t, int64_t> &ij_pos2);

    std::shared_ptr<GridMap> grid_map;
};

#endif // CTP_SWAP_THETA_STAR_HPP
