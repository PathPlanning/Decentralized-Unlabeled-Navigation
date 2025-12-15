#include "theta_planner.hpp"

ThetaPlanner::ThetaPlanner(const HolonomicAgentParams &ag_params, const std::shared_ptr<BaseAlgParams> &planner_params,
                           const std::vector<xt::xtensor<double, 1>> &goals, const std::shared_ptr<EnvironmentDescription> &env)
        : Planner(ag_params, planner_params, goals, env) {

    assert((grid_map != nullptr) && "Some error wih grid map. It's empty!");
    grid_map = std::static_pointer_cast<GridMap>(env);
    search = ThetaStar(grid_map);
    initialize();
}

xt::xtensor<double, 1> ThetaPlanner::findNext(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) {
    auto pos_cell = grid_map->getContainingCell(pos);
    auto goal_cell = grid_map->getContainingCell(goal);

    if (pos_cell.first == goal_cell.first and pos_cell.second == goal_cell.second) {
        return goal;
    }

    if (not grid_map->cellOnGrid(pos_cell) or grid_map->cellIsObstacle(pos_cell)) {
         std::cerr << "Some error in path planning. Incorrect position "
         			 "(obstacle or out of boundary!\n";
        return pos;
    }

    if (path_table.find(goal_cell) == path_table.end()) {
        std::cerr << "Some error in path table. Goal does not exists here!\n";
        return pos;
    }
    assert((path_table[goal_cell][pos_cell]->parent != nullptr) && "Some error in path finding!");

    auto next_cell = path_table[goal_cell][pos_cell]->parent->node_id;
    auto next = grid_map->getCenterPoint(next_cell);
    return next;
}

double ThetaPlanner::findLength(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) {

    auto pos_cell = grid_map->getContainingCell(pos);
    auto goal_cell = grid_map->getContainingCell(goal);

    if (path_table.find(goal_cell) == path_table.end()) {
        std::cerr << "Some error in path table. Goal does not exists here!\n";
        return -1.0;
    }

    assert((path_table.find(goal_cell) != path_table.end()) &&
    	   "Some error in path table. Goal does not exist here!");

    if (path_table[goal_cell].find(pos_cell) == path_table[goal_cell].end()) {
        std::cerr << "Some error in path planning. Incorrect position "
                     "(obstacle or out of boundary!\n";
        return -1.0;
    }

    auto grid_len = path_table[goal_cell][pos_cell]->len;
    return grid_len + xt::linalg::norm(pos - grid_map->getCenterPoint(pos_cell));
}

std::vector<double> ThetaPlanner::findAllLengths(const xt::xtensor<double, 1> &pos) { 
	
    std::vector<double> result;
	result.resize(goals.size());
	for (size_t goal_id = 0; goal_id < goals.size(); ++goal_id) {
		result[goal_id] = findLength(pos, goals[goal_id]);
	}
	return result;

}

void ThetaPlanner::initialize() {

    if (grid_map->getCellSize() < ag_params.size) {
        grid_map->inflate(ag_params.size);
    }

    for (auto &goal: goals) {
        auto goal_cell = grid_map->getContainingCell(goal);
        auto expanded = search.findPaths(goal_cell);
        path_table[goal_cell] = expanded;
    }
}


