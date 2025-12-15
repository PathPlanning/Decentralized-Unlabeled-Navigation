#include "simple_agent.hpp"

bool SimpleAgent::initialize() {
	auto min_dist = xt::linalg::norm(goals[0] - state.position);
	goal = goals[0];
	for (auto &curr_goal : goals) {
		auto dist = xt::linalg::norm(curr_goal - state.position);
		if (dist < min_dist) {
			min_dist = dist;
			goal = curr_goal;
		}
	}
	return true;
}

Message SimpleAgent::sendMessage() {
	auto message = Message();
	message.id = a_id;
	message.priority = a_id;
	message.size = ag_params.size;
	return message;
}

void SimpleAgent::updateNeighborsInfo(
	const std::vector<Message> &neighbors_info) {
	return;
}

void SimpleAgent::updateStateInfo(const State &new_state) { state = new_state; }

Action SimpleAgent::computeAction() {
	Action action;
	xt::xtensor<double, 1> goal_vec = goal - state.position;
	auto norm = xt::linalg::norm(goal_vec);
	action.velocity = goal_vec;
	action.goal = goal;
	if (norm > ag_params.vel_max) {
		action.velocity /= norm * ag_params.vel_max;
	}

	return action;
}
