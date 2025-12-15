#ifndef CTP_SWAP_GRID_MAP_HPP
#define CTP_SWAP_GRID_MAP_HPP
#include "xtensor/xtensor.hpp"
#include <cassert>
#include <memory>
#include <string>
#include <vector>
#include "environment_description.hpp"

class GridMap : public  EnvironmentDescription {
  public:
	GridMap() = delete;

	GridMap(const xt::xtensor<int8_t, 2> &grid, double cell_size);

	void inflate(double agent_radius);

    [[nodiscard]] std::vector<std::pair<int64_t, int64_t>>
    findNeighbors(const std::pair<int64_t, int64_t> &ij_pos) const;

	[[nodiscard]] bool cellIsObstacle(const std::pair<int64_t, int64_t> &ij_pos) const;

	[[nodiscard]] bool cellIsNotPref(const std::pair<int64_t, int64_t> &ij_pos) const;
	
	[[nodiscard]] bool cellOnGrid(const std::pair<int64_t, int64_t> &ij_pos) const;

	[[nodiscard]] bool cellIsTraversable(const std::pair<int64_t, int64_t> &ij_pos) const;

	[[nodiscard]] int64_t getHeight() const;

	[[nodiscard]] int64_t getWidth() const;

	[[nodiscard]] xt::xtensor<int8_t, 2> getGrid() const;

	[[nodiscard]] int8_t getCost(const std::pair<int64_t, int64_t> &ij_pos) const;

	[[nodiscard]] double getCellSize() const;


	[[nodiscard]] std::pair<int64_t, int64_t> getContainingCell(const xt::xtensor<double, 1> &xy_pos) const;

	[[nodiscard]] xt::xtensor<double, 1> getCenterPoint(const std::pair<int64_t, int64_t> &ij_pos) const;

  protected:
	double cell_size;
	int64_t height;
	int64_t width;
	xt::xtensor<int8_t, 2> grid;
};

#endif // CTP_SWAP_GRID_MAP_HPP
