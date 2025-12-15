#include "theta_star.hpp"


double
ThetaStar::computeDist(const std::pair<int64_t, int64_t> &ij_pos1, const std::pair<int64_t, int64_t> &ij_pos2) const {
    double di = static_cast<double>((ij_pos1.first - ij_pos2.first)) * grid_map->getCellSize();
    double dj = static_cast<double>((ij_pos1.second - ij_pos2.second)) * grid_map->getCellSize();
    return std::sqrt(di * di + dj * dj);
}

double
ThetaStar::computeCost(const std::pair<int64_t, int64_t> &ij_pos1, const std::pair<int64_t, int64_t> &ij_pos2) const {
    int64_t di = ij_pos1.first - ij_pos2.first;
    int64_t dj = ij_pos1.second - ij_pos2.second;

    if (grid_map->getCost(ij_pos1) > 1) {
        return grid_map->getCost(ij_pos1);
    }

    if (grid_map->getCost(ij_pos2) > 1) {
        return grid_map->getCost(ij_pos2);
    }

    if (std::abs(di) + std::abs(dj) == 1) {
        return 1.0;
    } else if (std::abs(di) + std::abs(dj) == 2) {
        return std::sqrt(2);
    }
    return std::sqrt(di * di + dj * dj);
}

std::unordered_map<std::pair<int64_t, int64_t>, std::shared_ptr<Node<std::pair<int64_t, int64_t>>>, GridPosHash>
ThetaStar::findPaths(const std::pair<int64_t, int64_t> &goal) {

    auto ast = SearchTree<std::pair<int64_t, int64_t>, GridPosHash>();
    auto curr_node = std::make_shared<Node<std::pair<int64_t, int64_t>>>();
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
        auto neighbors = grid_map->findNeighbors(curr_node->node_id);

        for (auto &neighbor: neighbors) {
            auto new_node = std::make_shared<Node<std::pair<int64_t, int64_t>>>();
            new_node->node_id = neighbor;
            if (not ast.wasExpanded(new_node)) {
                new_node->g = curr_node->g + computeCost(curr_node->node_id, new_node->node_id);
                new_node->len = curr_node->len + computeDist(curr_node->node_id, new_node->node_id);
                new_node->parent = curr_node;
                new_node = resetParent(new_node, new_node->parent);
                ast.addToOpen(new_node);
            }
        }
    }

    return ast.getExpanded();
}


bool ThetaStar::lineOfSight(const std::pair<int64_t, int64_t> &ij_pos1, const std::pair<int64_t, int64_t> &ij_pos2) {

    int64_t i1 = ij_pos1.first;
    int64_t j1 = ij_pos1.second;
    int64_t i2 = ij_pos2.first;
    int64_t j2 = ij_pos2.second;

    int64_t delta_i = std::abs(i1 - i2);
    int64_t delta_j = std::abs(j1 - j2);
    int64_t step_i = (i1 < i2 ? 1 : -1);
    int64_t step_j = (j1 < j2 ? 1 : -1);
    int64_t error = 0;
    int64_t i = i1;
    int64_t j = j1;
    if (delta_i == 0) {
        for (; j != j2; j += step_j)
            if (grid_map->cellIsNotPref({i, j}))
                return false;
        return true;
    } else if (delta_j == 0) {
        for (; i != i2; i += step_i)
            if (grid_map->cellIsNotPref({i, j}))
                return false;
        return true;
    }
    int64_t sep_value = delta_i * delta_i + delta_j * delta_j;
    if (delta_i > delta_j) {
        for (; i != i2; i += step_i) {
            if (grid_map->cellIsNotPref({i, j}))
                return false;
            if (grid_map->cellIsNotPref({i, j + step_j}))
                return false;
            error += delta_j;
            if (error >= delta_i) {
                if (((error << 1) - delta_i - delta_j) * ((error << 1) - delta_i - delta_j) < sep_value)
                    if (grid_map->cellIsNotPref({i + step_i, j}))
                        return false;
                if ((3 * delta_i - ((error << 1) - delta_j)) * (3 * delta_i - ((error << 1) - delta_j)) < sep_value)
                    if (grid_map->cellIsNotPref({i, j + 2 * step_j}))
                        return false;
                j += step_j;
                error -= delta_i;
            }
        }
        if (grid_map->cellIsNotPref({i, j}))
            return false;
    } else {
        for (; j != j2; j += step_j) {
            if (grid_map->cellIsNotPref({i, j}))
                return false;
            if (grid_map->cellIsNotPref({i + step_i, j}))
                return false;
            error += delta_i;
            if (error >= delta_j) {
                if (((error << 1) - delta_i - delta_j) * ((error << 1) - delta_i - delta_j) <
                    (delta_i * delta_i + delta_j * delta_j))
                    if (grid_map->cellIsNotPref({i, j + step_j}))
                        return false;
                if ((3 * delta_j - ((error << 1) - delta_i)) * (3 * delta_j - ((error << 1) - delta_i)) <
                    (delta_i * delta_i + delta_j * delta_j))
                    if (grid_map->cellIsNotPref({i + 2 * step_i, j}))
                        return false;
                i += step_i;
                error -= delta_j;
            }
        }
        if (grid_map->cellIsNotPref({i, j}))
            return false;
    }
    return true;
}

std::shared_ptr<Node<std::pair<int64_t, int64_t>>>
ThetaStar::resetParent(std::shared_ptr<Node<std::pair<int64_t, int64_t>>> current,
                       const std::shared_ptr<Node<std::pair<int64_t, int64_t>>> &parent) {
    if (parent->parent == nullptr)
        return current;
    if (current == parent->parent)
        return current;

    if (lineOfSight(parent->parent->node_id, current->node_id)) {
        current->g = parent->parent->g + computeCost(parent->parent->node_id, current->node_id);
        current->len = parent->parent->len + computeDist(parent->parent->node_id, current->node_id);
        current->parent = parent->parent;
        return current;
    }
    return current;
}