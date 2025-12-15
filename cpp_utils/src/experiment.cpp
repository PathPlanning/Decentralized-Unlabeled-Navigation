#include "experiment.hpp"

Experiment::Experiment(
	size_t agents_num, const std::vector<double> &sizes,
	const std::vector<double> &r_vis,
	const std::vector<std::vector<xt::xtensor<double, 1>>> &obstacles)
	: agents_num(agents_num), sizes(sizes) {

	rvo_sim = std::make_shared<RVO::RVOSimulator>();
	rvo_sim->setTimeStep(0.1);

	auto position = RVO::Vector2(0, 0);
	for (size_t a_id = 0; a_id < agents_num; ++a_id) {
		a_id = rvo_sim->addAgent(position, r_vis[a_id], agents_num, 1.0, 1.0,
								 sizes[a_id], 1.0);
	}

	std::vector<std::vector<RVO::Vector2>> obstacles_points;
	for (auto &obstacle : obstacles) {
		std::vector<RVO::Vector2> obstacle_rvo;
		for (auto &vertex : obstacle) {
			auto vertex_rvo = RVO::Vector2(vertex[0], vertex[1]);
			obstacle_rvo.push_back(vertex_rvo);
		}
		rvo_sim->addObstacle(obstacle_rvo);
	}
	rvo_sim->processObstacles();
}

void Experiment::updateState(const xt::xtensor<double, 2> &new_positions) {

	for (size_t a_id = 0; a_id < agents_num; ++a_id) {

		rvo_sim->setAgentPosition(
			a_id, RVO::Vector2(new_positions(a_id, 0), new_positions(a_id, 1)));
		rvo_sim->setAgentVelocity(a_id, RVO::Vector2(0, 0));

		rvo_sim->setAgentPrefVelocity(a_id, RVO::Vector2(0, 0));
	}
	rvo_sim->doStep();
}

std::vector<std::unordered_set<size_t>> Experiment::computeNeighbors() {

	auto neighbors = std::vector<std::unordered_set<size_t>>(agents_num);

	for (size_t a_id = 0; a_id < agents_num; ++a_id) {

		for (size_t n_num = 0; n_num < rvo_sim->getAgentNumAgentNeighbors(a_id);
			 ++n_num) {
			neighbors[a_id].insert(rvo_sim->getAgentAgentNeighbor(a_id, n_num));
		}
	}
	return neighbors;
}

std::vector<std::unordered_set<size_t>> Experiment::computeConnectedGroups() {

	auto neighbors = computeNeighbors();
	std::vector<std::unordered_set<size_t>> neighbors_groups;
	std::unordered_set<size_t> considered;

	for (size_t a_id = 0; a_id < agents_num; ++a_id) {

		if (considered.find(a_id) != considered.end()) {
			continue;
		}
		considered.insert(a_id);
		std::unordered_set<size_t> group;
		std::list<size_t> queue;
		queue.push_back(a_id);
		while (not queue.empty()) {
			auto curr = queue.front();
			queue.pop_front();
			group.insert(curr);
			considered.insert(curr);
			for (auto &n_id : neighbors[curr]) {
				if (group.find(n_id) != group.end()) {
					continue;
				}
				queue.push_back(n_id);
			}
		}
		neighbors_groups.push_back(group);
	}
	return neighbors_groups;
}

int64_t Experiment::checkCollisions() {

	int64_t collisions = 0;
	for (size_t a_id = 0; a_id < agents_num; ++a_id) {

		auto a_pos = rvo_sim->getAgentPosition(a_id);
		auto a_size = sizes[a_id];
		for (size_t n_num = 0; n_num < rvo_sim->getAgentNumAgentNeighbors(a_id);
			 ++n_num) {
			auto n_id = rvo_sim->getAgentAgentNeighbor(a_id, n_num);
			auto n_pos = rvo_sim->getAgentPosition(n_id);
			auto size_sum_sq = (a_size + sizes[n_id]) * (a_size + sizes[n_id]);
			auto dist_sq = RVO::absSq(n_pos - a_pos);
			if (size_sum_sq < dist_sq)
				break;
			collisions++;
		}
	}
	return collisions;
}

int64_t Experiment::checkCollisionsObstacles() {
	int64_t collisions = 0;
	for (size_t a_id = 0; a_id < agents_num; ++a_id) {

		auto a_pos = rvo_sim->getAgentPosition(a_id);
		auto a_size_sq = sizes[a_id] * sizes[a_id];


		for (size_t o_num = 0; o_num < rvo_sim->getAgentNumObstacleNeighbors(a_id);
			 ++o_num) {

            auto o_id = rvo_sim->getAgentObstacleNeighbor(a_id, o_num);
            auto o_pos = rvo_sim->getObstacleVertex(o_id);
            auto next_o_pos = rvo_sim->getObstacleVertex(rvo_sim->getNextObstacleVertexNo(o_id));



            float dist_sq = 0.0F;
            const float r = ((a_pos - o_pos) *
                            (next_o_pos - o_pos)) /
                            absSq(next_o_pos - o_pos);

            if (r < 0.0F) {
                dist_sq = absSq(a_pos - o_pos);
            } else if (r > 1.0F) {
                dist_sq = absSq(a_pos - next_o_pos);
            } else {
                dist_sq = absSq(a_pos - (o_pos +
                                            r * (next_o_pos - o_pos)));
            }

			if (a_size_sq < dist_sq)
				break;
			std::cout << a_id << "\n";
			collisions++;
		}
	}
	return collisions;
}
