#include "converters.hpp"

HolonomicAgentParams convertAgentParams(const py::object &ag_params) {
	auto new_ag_params = HolonomicAgentParams();
	new_ag_params.r_vis = py::getattr(ag_params, "r_vis").cast<double>();
	new_ag_params.size = py::getattr(ag_params, "size").cast<double>();
	new_ag_params.vel_max = py::getattr(ag_params, "vel_max").cast<double>();
	return new_ag_params;
}

std::shared_ptr<BaseAlgParams> convertAlgParams(const py::object &alg_params) {

    std::shared_ptr<BaseAlgParams> result;
	if (py::getattr(alg_params, "alg_name").cast<std::string>() == "orca_alg") {
		// std::cout << "ORCA!!!\n";

		auto new_ag_params = ORCAParams();
		new_ag_params.timestep = py::getattr(alg_params, "timestep").cast<double>();
		new_ag_params.sight_radius = py::getattr(alg_params, "sight_radius").cast<double>();
		new_ag_params.agents_max_num = py::getattr(alg_params, "agents_max_num").cast<int>();
		new_ag_params.time_boundary = py::getattr(alg_params, "time_boundary").cast<double>();
		new_ag_params.time_boundary_obst = py::getattr(alg_params, "time_boundary_obst").cast<double>();
		new_ag_params.r_eps = py::getattr(alg_params, "r_eps").cast<double>();
		new_ag_params.max_speed = py::getattr(alg_params, "max_speed").cast<double>();
        result = std::make_shared<ORCAParams>(new_ag_params);

	}
	if (py::getattr(alg_params, "alg_name").cast<std::string>() == "thetastar_alg") {
		// std::cout << "Theta!!!\n";
        result = std::make_shared<BaseAlgParams>(BaseAlgParams());
	}
	if (py::getattr(alg_params, "alg_name").cast<std::string>() == "direct_plan_alg") {
		// std::cout << "Direct plan!!!\n";
        result = std::make_shared<BaseAlgParams>(BaseAlgParams());
	}
    if (py::getattr(alg_params, "alg_name").cast<std::string>() == "visibility_plan_alg") {
        // std::cout << "Direct plan!!!\n";
        result = std::make_shared<BaseAlgParams>(BaseAlgParams());
    }
	if (py::getattr(alg_params, "alg_name").cast<std::string>() == "direct_follow_alg") {
		// std::cout << "Dirce follow!!!\n";
        result = std::make_shared<BaseAlgParams>(BaseAlgParams());
	}
	if (py::getattr(alg_params, "alg_name").cast<std::string>() == "simple_nav_alg") {
		// std::cout << "Simple nav!!!\n";
        result = std::make_shared<BaseAlgParams>(BaseAlgParams());
	}
	if (py::getattr(alg_params, "alg_name").cast<std::string>() == "ctp_swap_alg") {
		auto new_ag_params = CTPSWAPParams();
		new_ag_params.amapf = py::getattr(alg_params, "amapf").cast<bool>();
        result = std::make_shared<CTPSWAPParams>(new_ag_params);
	}

	return result;
}