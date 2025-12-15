#include "orca_follower.hpp"

ORCAFollower::ORCAFollower(HolonomicAgentParams ag_params,
                           std::shared_ptr<BaseAlgParams> follower_params,
                           const std::vector<std::vector<xt::xtensor<double, 1> > > &obstacles)
    : Follower(ag_params, follower_params, obstacles) {
    orca_params = std::static_pointer_cast<ORCAParams>(follower_params);
    initialize();
}

void ORCAFollower::initialize() {

    rvo_sim = std::make_shared<RVO::RVOSimulator>();

    rvo_sim->setTimeStep(orca_params->timestep);

    auto position = RVO::Vector2(0, 0);
    a_id = rvo_sim->addAgent(position, orca_params->sight_radius, orca_params->agents_max_num,
                             orca_params->time_boundary, orca_params->time_boundary_obst,
                             ag_params.size + orca_params->r_eps, orca_params->max_speed);

    std::vector<std::vector<RVO::Vector2> > obstacles_points;
    for (auto &obstacle: obstacles) {
        std::vector<RVO::Vector2> obstacle_rvo;
        for (auto &vertex: obstacle) {
            auto vertex_rvo = RVO::Vector2(vertex[0], vertex[1]);
            obstacle_rvo.push_back(vertex_rvo);
        }
        rvo_sim->addObstacle(obstacle_rvo);
    }
    rvo_sim->processObstacles();


}

void ORCAFollower::updateNeighborsInfo(std::vector<State> neighbors_states,
                                       std::vector<float> neighbors_sizes) {
    auto sim_ag_num = rvo_sim->getNumAgents();
    auto curr_ag_num = neighbors_states.size() + 1;

    if (sim_ag_num < curr_ag_num) {
        auto position = RVO::Vector2(0, 0);
        for (size_t i = sim_ag_num; i < curr_ag_num; ++i) {
            rvo_sim->addAgent(position, 1e-5, 0, 1e-5, 1e-5,
                              ag_params.size + orca_params->r_eps, orca_params->max_speed);
        }
    }
    for (size_t i = 1; i < curr_ag_num; ++i) {
        rvo_sim->setAgentPosition(i, RVO::Vector2(neighbors_states[i - 1].position[0],
                                                  neighbors_states[i - 1].position[1]));
        rvo_sim->setAgentVelocity(i, RVO::Vector2(neighbors_states[i - 1].velocity[0],
                                                  neighbors_states[i - 1].velocity[1]));
        rvo_sim->setAgentRadius(i, neighbors_sizes[i - 1] + orca_params->r_eps);
        rvo_sim->setAgentPrefVelocity(i, RVO::Vector2(0, 0));
    }

    if (sim_ag_num > curr_ag_num) {
        auto position = RVO::Vector2(1e5, 1e5);
        auto velocity = RVO::Vector2(0, 0);
        for (size_t i = curr_ag_num; i < sim_ag_num; ++i) {
            rvo_sim->setAgentPosition(i, position);
            rvo_sim->setAgentVelocity(i, velocity);
            rvo_sim->setAgentRadius(i, 1e-5);
        }
    }
}

Action ORCAFollower::findAction(const State &state, const xt::xtensor<double, 1> &waypoint) {
    rvo_sim->setAgentPosition(a_id, RVO::Vector2(state.position[0], state.position[1]));
    rvo_sim->setAgentVelocity(a_id, RVO::Vector2(state.velocity[0], state.velocity[1]));

    xt::xtensor<double, 1> waypoint_vec = waypoint - state.position;
    auto norm = xt::linalg::norm(waypoint_vec);
    xt::xtensor<double, 1> pref_vel = waypoint_vec;


    if (norm > 0.05) {
        pref_vel = (pref_vel / norm) * ag_params.vel_max;
    }

//    if (norm > ag_params.size) {
//        pref_vel /= norm * ag_params.vel_max;
//    }


    rvo_sim->setAgentPrefVelocity(a_id, RVO::Vector2(pref_vel[0], pref_vel[1]));
    rvo_sim->doStep();

    auto new_velocity = rvo_sim->getAgentVelocity(a_id);
    Action action;
    action.velocity[0] = new_velocity.x();
    action.velocity[1] = new_velocity.y();
    return action;
}


