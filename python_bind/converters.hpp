
#ifndef CTP_SWAP_CONVERTERS_HPP
#define CTP_SWAP_CONVERTERS_HPP

#include <stdexcept>
#include <memory>

#include "../include/params.hpp"
#include "../include/orca_follower.hpp"
#include "../include/ctpswap_agent.hpp"
#include <pybind11/pybind11.h>
#include "xtensor-python/pyarray.hpp"

namespace py = pybind11;

HolonomicAgentParams convertAgentParams(const py::object &ag_params);

std::shared_ptr<BaseAlgParams> convertAlgParams(const py::object &alg_params);


// py::object getValue(const py::dict& data, const std::string &key, const py::object &default_value);

// template <typename T>
// T getValue(const py::dict& data, const std::string &key, const T &default_value);



#endif // CTP_SWAP_CONVERTERS_HPP