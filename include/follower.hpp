#ifndef CTP_SWAP_FOLLOWER_HPP
#define CTP_SWAP_FOLLOWER_HPP

#include <cstdint>
#include <xtensor/xtensor.hpp>

#include "action.hpp"
#include "params.hpp"
#include "state.hpp"

class Follower {
  public:

	Follower() = delete;
	Follower(HolonomicAgentParams ag_params,
			 std::shared_ptr<BaseAlgParams> follower_params,
			 const std::vector<std::vector<xt::xtensor<double, 1>>>& obstacles)
		: ag_params(ag_params), follower_params(follower_params), obstacles(obstacles) {};



	virtual Action findAction(const State &state,
							  const xt::xtensor<double, 1> &waypoint) = 0;
	virtual void updateNeighborsInfo(std::vector<State> neighbors_states,
									 std::vector<float> neighbors_sizes) = 0;

  protected:
	HolonomicAgentParams ag_params;
	std::shared_ptr<BaseAlgParams> follower_params;
	std::vector<std::vector<xt::xtensor<double, 1>>> obstacles;
};

#endif // CTP_SWAP_FOLLOWER_HPP