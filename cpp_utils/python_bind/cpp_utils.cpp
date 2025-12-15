#include <cstdint>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#define FORCE_IMPORT_ARRAY
#include "../include/experiment.hpp"
#include <xtensor-python/pytensor.hpp>
#include <xtensor/xtensor.hpp>

namespace py = pybind11;

PYBIND11_MODULE(experiment_utils, m) {
	xt::import_numpy();

	py::class_<Experiment, std::shared_ptr<Experiment>>(m, "Experiment")
		.def(py::init<size_t, std::vector<double>, std::vector<double>,
					  std::vector<std::vector<xt::xtensor<double, 1>>>>())
		.def("update_state", &Experiment::updateState)
		.def("compute_neighbors", &Experiment::computeNeighbors)
		.def("compute_connected_groups", &Experiment::computeConnectedGroups)
		.def("check_collisions", &Experiment::checkCollisions)
		.def("check_collisions_obstacles", &Experiment::checkCollisionsObstacles);
}
