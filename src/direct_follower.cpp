#include "direct_follower.hpp"

Action DirectFollower::findAction(const State &state,
								  const xt::xtensor<double, 1> &waypoint) {

	Action action;
	xt::xtensor<double, 1> waypoint_vec = waypoint - state.position;
	auto norm = xt::linalg::norm(waypoint_vec);
	action.velocity = waypoint_vec;
	if (norm > ag_params.vel_max) {
		action.velocity /= norm * ag_params.vel_max;
	}

	return action;
}


void DirectFollower::updateNeighborsInfo(std::vector<State> neighbors_states,
										 std::vector<float> neighbors_sizes) {}