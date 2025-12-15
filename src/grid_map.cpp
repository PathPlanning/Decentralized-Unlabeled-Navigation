#include "grid_map.hpp"

GridMap::GridMap(const xt::xtensor<int8_t, 2> &grid, double cell_size) {
    this->grid = grid;
    this->cell_size = cell_size;
    this->height = static_cast<int64_t>(grid.shape(0));
    this->width = static_cast<int64_t>(grid.shape(1));
}

void GridMap::inflate(double agent_radius) {

    xt::xtensor<int8_t, 2> new_grid = xt::zeros_like(grid);

    for (int64_t i = -1; i <= height; ++i) {
        for (int64_t j = -1; j <= width; ++j) {
            if (i != -1 and j != -1 and i != height and j != width and cellIsTraversable({i, j})) {
                continue;
            }

            auto inflation_cells = static_cast<int64_t>(ceil(agent_radius / cell_size));

            auto inflation_cells_sq = inflation_cells * inflation_cells;

            for (int64_t di = -inflation_cells; di <= inflation_cells; ++di) {
                for (int64_t dj = -inflation_cells; dj <= inflation_cells; ++dj) {
                    int64_t new_i = i + di;
                    int64_t new_j = j + dj;
                    if (new_i >= 0 and new_i < height and new_j >= 0 and new_j < width) {
                        int64_t distance = di * di + dj * dj;
                        if (distance <= inflation_cells_sq) {

                            if (grid[{new_i, new_j}] == 1) {
                                new_grid[{new_i, new_j}] = 1;
                            } else {
                                new_grid[{new_i, new_j}] = 100;
                            }
                        }
                    }
                }
            }
        }
    }

    grid = new_grid;
}

bool GridMap::cellIsObstacle(const std::pair<int64_t, int64_t> &ij_pos) const {
    assert(cellOnGrid(ij_pos) && "Some error in map. Cell is outside map!");
    int8_t cell_value = grid[{ij_pos.first, ij_pos.second}];
    return cell_value == 1;
}

bool GridMap::cellIsNotPref(const std::pair<int64_t, int64_t> &ij_pos) const {
    assert(cellOnGrid(ij_pos) && "Some error in map. Cell is outside map!");
    auto cell_value = grid(ij_pos.first, ij_pos.second);
    return (cell_value != 0);
}

bool GridMap::cellOnGrid(const std::pair<int64_t, int64_t> &ij_pos) const {
    return (ij_pos.first >= 0 and ij_pos.first < height and ij_pos.second >= 0 and ij_pos.second < width);
}

bool GridMap::cellIsTraversable(const std::pair<int64_t, int64_t> &ij_pos) const {
    assert(cellOnGrid(ij_pos) && "Some error in map. Cell is outside map!");
    int8_t cell_value = grid[{ij_pos.first, ij_pos.second}];
    return (cell_value != 1);
}

int64_t GridMap::getHeight() const { return height; }

int64_t GridMap::getWidth() const { return width; }

xt::xtensor<int8_t, 2> GridMap::getGrid() const { return grid; }

int8_t GridMap::getCost(const std::pair<int64_t, int64_t> &ij_pos) const {
    return grid[{ij_pos.first, ij_pos.second}];
}

double GridMap::getCellSize() const { return cell_size; }

std::pair<int64_t, int64_t> GridMap::getContainingCell(const xt::xtensor<double, 1> &xy_pos) const {

    int64_t i, j;
    i = height - 1 - static_cast<int64_t>(xy_pos[1] / cell_size);
    j = static_cast<int64_t>(xy_pos[0] / cell_size);
    assert(cellOnGrid({i, j}) && "Some error in map. Point is outside map!");
    return {i, j};
}

xt::xtensor<double, 1> GridMap::getCenterPoint(const std::pair<int64_t, int64_t> &ij_pos) const {
    return {(static_cast<double >(ij_pos.second) * cell_size + cell_size / 2),
            (static_cast<double >(height) - 1 - static_cast<double >(ij_pos.first)) * cell_size + cell_size / 2};
}

std::vector<std::pair<int64_t, int64_t>> GridMap::findNeighbors(const std::pair<int64_t, int64_t> &ij_pos) const {
    std::vector<std::pair<int64_t, int64_t>> deltas = {{0,  1},
                                                       {1,  0},
                                                       {0,  -1},
                                                       {-1, 0},
                                                       {1,  1},
                                                       {1,  -1},
                                                       {-1, 1},
                                                       {-1, -1}};

    std::vector<std::pair<int64_t, int64_t>> neighbors;
    std::pair<int64_t, int64_t> neighbor;

    for (auto &delta: deltas) {
        auto di = delta.first;
        auto dj = delta.second;
        neighbor = {ij_pos.first + di, ij_pos.second + dj};

        if (not cellOnGrid(neighbor) or cellIsObstacle(neighbor))
            continue;

        if (di != 0 and dj != 0) {
            if (cellIsObstacle({ij_pos.first, ij_pos.second + dj}) or
                cellIsObstacle({ij_pos.first + di, ij_pos.second})) {
                continue;
            }
        }
        neighbors.push_back(neighbor);
    }
    return neighbors;
}
