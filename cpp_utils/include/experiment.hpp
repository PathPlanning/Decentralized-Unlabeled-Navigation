#ifndef EXPERIMENT_UTILS_EXPERIMENT_HPP
#define EXPERIMENT_UTILS_EXPERIMENT_HPP
#include <../libs/rvo2/src/RVOSimulator.h>
#include <../libs/rvo2/src/Vector2.h>

#include <cassert>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <list>

#include "xtensor/xtensor.hpp"


class Experiment {

  public:
	Experiment(size_t agents_num, const std::vector<double> &sizes, const std::vector<double> &r_vis, const std::vector<std::vector<xt::xtensor<double, 1>>> &obstacles);
    void updateState(const xt::xtensor<double, 2> &new_positions);
    std::vector<std::unordered_set<size_t>> computeNeighbors();
    std::vector<std::unordered_set<size_t>> computeConnectedGroups();
    int64_t checkCollisions();
    int64_t checkCollisionsObstacles();



  private:
  	std::shared_ptr<RVO::RVOSimulator> rvo_sim;
    size_t agents_num;
    std::vector<double> sizes;
};

#endif // EXPERIMENT_UTILS_EXPERIMENT_HPP
