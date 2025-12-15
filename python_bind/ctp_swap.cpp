#include <cstdint>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define FORCE_IMPORT_ARRAY

#include "../include/action.hpp"
#include "../include/ctpswap_agent.hpp"
#include "../include/message.hpp"
#include "../include/params.hpp"
#include "../include/simple_agent.hpp"
#include "../include/state.hpp"
#include "../include/direct_follower.hpp"
#include "../include/direct_planner.hpp"
#include "../include/follower.hpp"
#include "../include/grid_map.hpp"
#include "../include/orca_follower.hpp"
#include "../include/planner.hpp"
#include "../include/theta_planner.hpp"
#include "../include/environment_description.hpp"
#include "../include/visibility_graph.hpp"
#include "../include/visibility_planner.hpp"


#include "converters.hpp"

#include <xtensor-python/pytensor.hpp>
#include <xtensor/xtensor.hpp>

namespace py = pybind11;

PYBIND11_MODULE(_ctp_swap, m) {
    xt::import_numpy();
    py::class_<HolonomicAgentParams, std::shared_ptr<HolonomicAgentParams>>(m, "HolonomicAgentParams");

    py::class_<BaseAlgParams, std::shared_ptr<BaseAlgParams>>(m, "BaseAlgParams");

    py::class_<Message, std::shared_ptr<Message>>(m, "Message").def(py::init<>()).def_readwrite("goal", &Message::goal);


    py::class_<EnvironmentDescription, std::shared_ptr<EnvironmentDescription>>(m, "EnvironmentDescription");


    py::class_<GridMap, EnvironmentDescription, std::shared_ptr<GridMap>>(m, "GridMap").def(
            py::init<xt::xtensor<int8_t, 2>, double>()).def("inflate", &GridMap::inflate).def("get_map",
                                                                                              &GridMap::getGrid);

    py::class_<VisibilityGraph, EnvironmentDescription, std::shared_ptr<VisibilityGraph>>(m, "VisibilityGraph").def(
            py::init<std::vector<std::vector<xt::xtensor<double, 1>>>, std::vector<xt::xtensor<double, 1>>, double>());


    py::class_<Planner, std::shared_ptr<Planner>>(m, "Planner").def("find_next", &Planner::findNext);

    py::class_<DirectPlanner, Planner, std::shared_ptr<DirectPlanner>>(m, "DirectPlanner").def(
            py::init<HolonomicAgentParams, std::shared_ptr<BaseAlgParams>, std::vector<xt::xtensor<double, 1>>, std::shared_ptr<EnvironmentDescription>>());

    py::class_<ThetaPlanner, Planner, std::shared_ptr<ThetaPlanner>>(m, "ThetaPlanner").def(
            py::init<HolonomicAgentParams, std::shared_ptr<BaseAlgParams>, std::vector<xt::xtensor<double, 1>>, std::shared_ptr<EnvironmentDescription>>());

    py::class_<VisibilityPlanner, Planner, std::shared_ptr<VisibilityPlanner>>(m, "VisibilityPlanner").def(
            py::init<HolonomicAgentParams, std::shared_ptr<BaseAlgParams>, std::vector<xt::xtensor<double, 1>>, std::shared_ptr<EnvironmentDescription>>());

    py::class_<Follower, std::shared_ptr<Follower>>(m, "Follower");

    py::class_<DirectFollower, Follower, std::shared_ptr<DirectFollower>>(m, "DirectFollower").def(
            py::init<HolonomicAgentParams, std::shared_ptr<BaseAlgParams>, std::vector<std::vector<xt::xtensor<double, 1>>>>());

    py::class_<ORCAFollower, Follower, std::shared_ptr<ORCAFollower>>(m, "ORCAFollower").def(
            py::init<HolonomicAgentParams, std::shared_ptr<BaseAlgParams>, std::vector<std::vector<xt::xtensor<double, 1>>>>());

    py::class_<State, std::shared_ptr<State>>(m, "State").def(py::init<>()).def_readwrite("position",
                                                                                          &State::position).def_readwrite(
            "velocity", &State::velocity);

    py::class_<Action, std::shared_ptr<Action>>(m, "Action").def(py::init<>()).def_readwrite("velocity",
                                                                                             &Action::velocity).def_readwrite(
            "goal", &Action::goal);

    py::class_<Agent>(m, "Agent").def("initialize", &Agent::initialize).def("update_state_info",
                                                                            &Agent::updateStateInfo).def("send_message",
                                                                                                         &Agent::sendMessage).def(
            "update_neighbors_info", &Agent::updateNeighborsInfo).def("compute_action", &Agent::computeAction);

    py::class_<SimpleAgent, Agent>(m, "SimpleAgent").def(
            py::init<uint64_t, State, std::vector<xt::xtensor<double, 1>>, HolonomicAgentParams, std::shared_ptr<BaseAlgParams>, std::shared_ptr<Planner>, std::shared_ptr<Follower>>());;

    py::class_<CTPSWAPAgent, Agent>(m, "CTPSWAPAgent").def(
            py::init<uint64_t, State, std::vector<xt::xtensor<double, 1>>, HolonomicAgentParams, std::shared_ptr<BaseAlgParams>, std::shared_ptr<Planner>, std::shared_ptr<Follower>>());

    m.def("convert_agent_params", &convertAgentParams, "Converts agent's params");

    m.def("convert_alg_params", &convertAlgParams, "Coverts algorithm params");
}
