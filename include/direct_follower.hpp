#ifndef CTP_SWAP_DIRECT_FOLLOWER_HPP
#define CTP_SWAP_DIRECT_FOLLOWER_HPP

#include <cstdint>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xtensor.hpp>

#include "action.hpp"
#include "follower.hpp"
#include "state.hpp"

struct DirectFollowerParams : public BaseAlgParams {
};


class DirectFollower : public Follower {
  public:
	DirectFollower(HolonomicAgentParams ag_params,
				   std::shared_ptr<BaseAlgParams> follower_params, 
				   const std::vector<std::vector<xt::xtensor<double, 1>>>& obstacles)
		: Follower(ag_params, follower_params, obstacles) {};


	Action findAction(const State &state,
					  const xt::xtensor<double, 1> &waypoint) override;

	void updateNeighborsInfo(std::vector<State> neighbors_states,
							 std::vector<float> neighbors_sizes) override;
};

#endif // CTP_SWAP_DIRECT_FOLLOWER_HPP