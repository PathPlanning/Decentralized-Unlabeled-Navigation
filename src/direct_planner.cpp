#include "direct_planner.hpp"

xt::xtensor<double, 1> DirectPlanner::findNext(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) {

	return goal;
}

double DirectPlanner::findLength(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) {
	return xt::linalg::norm(goal - pos);
}

std::vector<double> DirectPlanner::findAllLengths(const xt::xtensor<double, 1> &pos) {
	std::vector<double> result;
	result.resize(goals.size());
	for (size_t goal_id = 0; goal_id < goals.size(); ++goal_id) {
		result[goal_id] = xt::linalg::norm(goals[goal_id] - pos);
	}
	return result;
}
