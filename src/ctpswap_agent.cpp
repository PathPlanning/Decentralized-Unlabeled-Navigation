#include "ctpswap_agent.hpp"

bool CTPSWAPAgent::initialize() {
    xt::random::seed(1234); // TODO: as parameter
    buffer = std::list<double>(20, 1.0); // TODO: size os param, speed os param
    sum_speed = 1.0 * 20; // TODO: speed os param

    if (ctp_swap_params->amapf) {
        auto min_dist = planner->findLength(state.position, goals[0]);
        goal = goals[0];
        goal_id = 0;
        for (size_t curr_id = 0; curr_id < goals.size(); ++curr_id) {
            auto curr_goal = goals[curr_id];
            auto dist = planner->findLength(state.position, curr_goal);
            if (dist < min_dist) {
                min_dist = dist;
                goal = curr_goal;
                goal_id = curr_id;
            }
        }
    } else {
        goal = goals[a_id];
        goal_id = a_id;
    }

    goals_priority_table = std::vector<int64_t>(goals.size(), -1);
    goals_priority_table[goal_id] = priority;

    return true;
}

Message CTPSWAPAgent::sendMessage() {
    auto message = Message();
    message.id = a_id;
    message.priority = priority;
    message.state = state;
    message.goal = goal;
    message.goal_id = goal_id;
    message.size = ag_params.size;
    message.goals_priority_table = goals_priority_table;
    // message.goals_lengths = planner->findAllLengths(state.position);
    return message;
}

void CTPSWAPAgent::updateNeighborsInfo(
    const std::vector<Message> &neighbors_info) {
    std::vector<State> neighbors_states(neighbors_info.size());
    std::vector<float> neighbors_sizes(neighbors_info.size());

    // current_neighbors_length.clear();
    // current_neighbors_length.reserve(neighbors_info.size());
    curent_goal_assignment.clear();
    curent_goal_assignment.reserve(neighbors_info.size());
    curent_priority_assignment.clear();
    curent_priority_assignment.reserve(neighbors_info.size());
    curent_positions.clear();
    curent_positions.reserve(neighbors_info.size());

    for (size_t i = 0; i < neighbors_info.size(); ++i) {
        curent_goal_assignment[neighbors_info[i].id] =
                neighbors_info[i].goal_id;
        curent_priority_assignment[neighbors_info[i].id] =
                neighbors_info[i].priority;
        curent_positions[neighbors_info[i].id] =
                neighbors_info[i].state.position;


        for (size_t curr_id = 0; curr_id < goals.size(); ++curr_id) {
            int64_t best_priority = -1;
            for (size_t j = 0; j < neighbors_info.size(); ++j) {
                auto curr_priority =
                        neighbors_info[j].goals_priority_table[curr_id];
                if (curr_priority > best_priority) {
                    best_priority = curr_priority;
                }
            }
            goals_priority_table[curr_id] = best_priority;
        }
    }

    for (size_t i = 0; i < neighbors_info.size(); ++i) {
        if (neighbors_info[i].id == a_id) {
            continue;
        }
        neighbors_states[i] = neighbors_info[i].state;
        neighbors_sizes[i] = neighbors_info[i].size;
    }
    follower->updateNeighborsInfo(neighbors_states, neighbors_sizes);
}

void CTPSWAPAgent::updateStateInfo(const State &new_state) {
    state = new_state;
    step++;
}

Action CTPSWAPAgent::computeAction() {
    size_t delta = 20;
    if (ctp_swap_params->amapf and step % delta == 0) {
        updateGoals();
    }

    auto waypoint = planner->findNext(state.position, goal);

    xt::xtensor<double, 1> waypoint_vec = waypoint - state.position;
    auto wv_norm = xt::linalg::norm(waypoint_vec);

    xt::xtensor<double, 1> pref_vel_random;


    //    if ( ( waypoint(0) == goal(0) and waypoint(1) == goal(1) and wv_norm < 0.1)) {
    //   pref_vel_random = xt::random::randn<double>({1, 2}, 0, 0.2 * wv_norm);
    //} else {

    if (sum_speed / 20.0 > 0.1) {
        pref_vel_random = xt::random::randn<double>({2}, 0, 0.2 * wv_norm); // TODO: as parameter
    } else {
        pref_vel_random = xt::random::randn<double>({2}, 0, 10.0 * wv_norm); // TODO: as parameter
    }
    //	}
    waypoint = waypoint + pref_vel_random;
    auto action = follower->findAction(state, waypoint);
    action.goal = goal;
    computeAvg(action);
    return action;
}

void CTPSWAPAgent::updateGoals() {
    for (int64_t n_id = goals.size() - 1; n_id >= 0; n_id--) {
        if (curent_goal_assignment.find(n_id) == curent_goal_assignment.end()) {
            continue;
        }

        auto agent_goal_id = curent_goal_assignment[n_id];
        auto agent_priority = curent_priority_assignment[n_id];
        auto agent_position = curent_positions[n_id];

        if (agent_priority >= goals_priority_table[agent_goal_id]) {
            continue;
        }

        xt::xtensor<double, 1> closest_goal_pos = {-1.0, -1.0};
        int64_t closest_goal_id = -1;
        double closest_goal_dist = std::numeric_limits<double>::infinity();

        for (size_t g_id = 0; g_id < goals.size(); ++g_id) {
            auto goal_priority = goals_priority_table[g_id];
            if (agent_priority < goal_priority) {
                continue;
            }

            // auto goal_dist = current_neighbors_length[n_id][g_id];


            auto goal_dist = planner->findLength(agent_position, goals[g_id]);

            if (closest_goal_dist < goal_dist) {
                continue;
            }
            closest_goal_pos = goal;
            closest_goal_id = g_id;
            closest_goal_dist = goal_dist;
        }

        // assert not np.all(closest_goal == np.array((-1, -1), dtype=np.int32))
        // assert closest_goal_dist < np.inf

        curent_goal_assignment[n_id] = closest_goal_id;
        goals_priority_table[closest_goal_id] = agent_priority;
    }

    double dist_a1_g1, dist_a2_g1, dist_a1_g2, dist_a2_g2;
    double sum_before, sum_after;
    bool swappable;
    for (int64_t n_id = goals.size() - 1; n_id >= 0; n_id--) {
        if (curent_goal_assignment.find(n_id) == curent_goal_assignment.end()) {
            continue;
        }

        auto agent_goal_id = curent_goal_assignment[n_id];
        auto agent_priority = curent_priority_assignment[n_id];
        auto agent_position = curent_positions[n_id];

        for (int64_t n2_id = n_id - 1; n2_id >= 0; --n2_id) {
            if (curent_goal_assignment.find(n2_id) ==
                curent_goal_assignment.end()) {
                continue;
            }

            auto agent2_goal_id = curent_goal_assignment[n2_id];
            auto agent2_priority = curent_priority_assignment[n2_id];
            auto agent2_position = curent_positions[n2_id];

            swappable = reachedAgentLieOnPath(agent_position, agent2_position,
                                              goals[agent_goal_id],
                                              goals[agent2_goal_id]) or
                        reachedAgentLieOnPath(agent2_position, agent_position,
                                              goals[agent2_goal_id],
                                              goals[agent_goal_id]);


            if (not swappable) {
                // auto goal_dist = current_neighbors_length[n_id][g_id];

                // dist_a1_g1 = current_neighbors_length[n_id][agent_goal_id];
                dist_a1_g1 = planner->findLength(agent_position, goals[agent_goal_id]);
                // dist_a2_g1 = current_neighbors_length[n2_id][agent_goal_id];
                dist_a2_g1 = planner->findLength(agent2_position, goals[agent_goal_id]);
                // dist_a1_g2 = current_neighbors_length[n_id][agent2_goal_id];
                dist_a1_g2 = planner->findLength(agent_position, goals[agent2_goal_id]);
                // dist_a2_g2 = current_neighbors_length[n2_id][agent2_goal_id];
                dist_a2_g2 = planner->findLength(agent2_position, goals[agent2_goal_id]);

                sum_before = dist_a1_g1 + dist_a2_g2;
                sum_after = dist_a2_g1 + dist_a1_g2;

                //                 if (n_id == 6 and n2_id == 10 or n_id == 10 and n2_id == 6) {
                //                     std::cout << "a1: " << n_id << " a2: " << n2_id << std::endl;
                //                     std::cout << "p1: " << agent_position << " g1: " << goals[agent_goal_id] << std::endl;
                //                     std::cout << "p2: " << agent2_position << " g2: " << goals[agent2_goal_id] << std::endl;
                //
                //                     std::cout << dist_a1_g1 << " " << dist_a2_g2 << "\n";
                //                     std::cout << dist_a1_g2 << " " << dist_a2_g1 << "\n";
                //                     std::cout << sum_before << " " << sum_after << "\n";
                //
                //                     auto waypoint_a1 = planner->findNext(agent2_position, goals[agent2_goal_id]);
                //                     std::cout << waypoint_a1 << "\n";
                //                     std::cout << std::endl;
                //
                //                 }

                swappable = swappable or (sum_before > sum_after);
            }

            if (swappable) {
                std::swap(agent_goal_id, agent2_goal_id);
                std::swap(agent_priority, agent2_priority);
                curent_goal_assignment[n_id] = agent_goal_id;
                curent_goal_assignment[n2_id] = agent2_goal_id;

                curent_priority_assignment[n_id] = agent_priority;
                curent_priority_assignment[n2_id] = agent2_priority;
            }
        }
    }

    this->goal_id = curent_goal_assignment[a_id];
    this->goal = goals[goal_id];
    this->priority = curent_priority_assignment[a_id];
}

bool CTPSWAPAgent::reachedAgentLieOnPath(
    const xt::xtensor<double, 1> &pos_a1, const xt::xtensor<double, 1> &pos_a2,
    const xt::xtensor<double, 1> &goal_a1,
    const xt::xtensor<double, 1> &goal_a2) const {
    bool a2_on_goal = xt::linalg::norm(goal_a2 - pos_a2) < 3e-1;
    if (not a2_on_goal) {
        return false;
    }

    auto point_seg_distance = [](const xt::xtensor<double, 1> &l_point1,
                                 const xt::xtensor<double, 1> &l_point2,
                                 const xt::xtensor<double, 1> &point) {
        xt::xtensor<double, 1> v = l_point2 - l_point1;
        xt::xtensor<double, 1> w = point - l_point1;
        auto c1 = xt::linalg::vdot(w, v);

        if (c1 <= 0.0f) {
            return xt::linalg::norm(point - l_point1);
        }
        double c2 = xt::linalg::vdot(v, v);
        if (c2 <= c1) {
            return xt::linalg::norm(point - l_point2);
        }

        double b = c1 / c2;
        const xt::xtensor<double, 1> p_b = l_point1 + v * b;
        return xt::linalg::norm(point - p_b);
    };

    auto waypoint_a1 = planner->findNext(pos_a1, goal_a1);

    return point_seg_distance(pos_a1, waypoint_a1, goal_a2) < 0.4;
}


void CTPSWAPAgent::computeAvg(const Action &action) {
    // std::cout << "before compute " << sum_speed << std::endl;
    const auto old_speed = buffer.front();
    buffer.pop_front();
    sum_speed -= old_speed;
    const double new_speed = xt::linalg::norm(action.velocity);
    buffer.push_back(new_speed);
    sum_speed += new_speed;
    // std::cout << "after compute " << sum_speed << std::endl;
}
