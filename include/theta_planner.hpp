#ifndef CTP_SWAP_THETA_PLANNER_HPP
#define CTP_SWAP_THETA_PLANNER_HPP

#include <cassert>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <xtensor-blas/xlinalg.hpp>

#include <xtensor/xtensor.hpp>

#include "planner.hpp"
#include "theta_star.hpp"

class ThetaPlanner : public Planner {

  public:
	ThetaPlanner(const HolonomicAgentParams &ag_params, const std::shared_ptr<BaseAlgParams> &planner_params,
				 const std::vector<xt::xtensor<double, 1>> &goals, const std::shared_ptr<EnvironmentDescription> &env);

	xt::xtensor<double, 1> findNext(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) override;

	double findLength(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) override;

	std::vector<double> findAllLengths(const xt::xtensor<double, 1> &pos) override;

  protected:
	void initialize();

	std::shared_ptr<GridMap> grid_map;
	ThetaStar search;
	std::unordered_map<std::pair<uint64_t, uint64_t>,
					   std::unordered_map<std::pair<int64_t, int64_t>,
										  std::shared_ptr<Node<std::pair<int64_t, int64_t>>>, GridPosHash>,
					   GridPosHash>
		path_table;
};

#endif // CTP_SWAP_THETA_PLANNER_HPP
