#include "visibility_planner.hpp"

VisibilityPlanner::VisibilityPlanner(const HolonomicAgentParams &ag_params,
                                     const std::shared_ptr<BaseAlgParams> &planner_params,
                                     const std::vector<xt::xtensor<double, 1> > &goals,
                                     const std::shared_ptr<EnvironmentDescription> &env)
    : Planner(ag_params, planner_params, goals, env) {
    graph = std::static_pointer_cast<VisibilityGraph>(env);
    assert((graph != nullptr) && "Some error wih vis. graph. It's empty!");
    search = Dijkstra(graph);
    initialize();
}

void VisibilityPlanner::initialize() {
    // auto start = std::chrono::system_clock::now();
    graph->initialize();
    // auto end = std::chrono::system_clock::now();
    // auto init_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //
    // std::cout << "Initialization time: " << init_time.count() / 1000.0 << " seconds" << std::endl;
    // std::cout << graph->getPointsCount() << std::endl;

    // start = std::chrono::system_clock::now();
    // for (int64_t g_id = 0; g_id < goals.size(); ++g_id) {
    //     goals_set.insert(g_id);
    // }
    path_table.resize(goals.size());
    for (int64_t g_id = 0; g_id < goals.size(); ++g_id) {
        point_index[goals[g_id]] = g_id;
        // auto search_start = std::chrono::system_clock::now();
        const auto expanded = search.findPaths(g_id);
        // auto search_end = std::chrono::system_clock::now();
        // auto search_time = std::chrono::duration_cast<std::chrono::milliseconds>(search_end - search_start);
        // std::cout << "Single search time  " << search_time.count() / 1000.0 << " seconds" << std::endl;
        path_table[g_id] = expanded;
    }

    // end = std::chrono::system_clock::now();
    // auto pathplanning_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //
    // std::cout << "Path planning time: " << pathplanning_time.count() / 1000.0 << " seconds" << std::endl;
}

double VisibilityPlanner::findLength(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) {
    auto points_ids = graph->getVisible(pos);

    auto expanded = path_table[point_index[goal]];

    double min_dist = std::numeric_limits<double>::max();
    for (auto point_id: points_ids) {
        if (expanded.find(point_id) == expanded.end()) {
            continue;
        }

        auto curr_dist = expanded[point_id]->len + xt::linalg::norm(graph->getPoint(point_id) - pos);
        if (curr_dist < min_dist) {
            min_dist = curr_dist;
        }
    }
    return min_dist;
}

std::vector<double> VisibilityPlanner::findAllLengths(const xt::xtensor<double, 1> &pos) {
    auto points_ids = graph->getVisible(pos);

    std::vector<double> result;
    result.resize(goals.size());
    for (size_t goal_id = 0; goal_id < goals.size(); ++goal_id) {
        auto expanded = path_table[goal_id];
        double min_dist = std::numeric_limits<double>::max();
        for (auto point_id: points_ids) {
            if (expanded.find(point_id) == expanded.end()) {
                continue;
            }

            auto curr_dist = expanded[point_id]->len + xt::linalg::norm(graph->getPoint(point_id) - pos);
            if (curr_dist < min_dist) {
                min_dist = curr_dist;
            }
        }
        result[goal_id] = min_dist;
    }
    return result;
}

xt::xtensor<double, 1> VisibilityPlanner::findNext(const xt::xtensor<double, 1> &pos,
                                                   const xt::xtensor<double, 1> &goal) {
    auto points_ids = graph->getVisible(pos);
    auto expanded = path_table[point_index[goal]];

    double min_dist = std::numeric_limits<double>::max();
    int64_t v_min = -1;
    for (auto point_id: points_ids) {
        if (expanded.find(point_id) == expanded.end()) {
            continue;
        }

        auto curr_dist = expanded[point_id]->len + xt::linalg::norm(graph->getPoint(point_id) - pos);
        if (curr_dist < min_dist) {
            min_dist = curr_dist;
            v_min = point_id;
        }
    }
    if (v_min == -1) {
        return {-1, -1};
    }
    return graph->getPoint(v_min);
}
