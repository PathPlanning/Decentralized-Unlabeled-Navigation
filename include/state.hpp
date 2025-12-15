#ifndef CTP_SWAP_STATE_HPP
#define CTP_SWAP_STATE_HPP

#include <cstdint>

#include "xtensor/xtensor.hpp"

struct State {
	xt::xtensor<double, 1> position = {0.0, 0.0};
	xt::xtensor<double, 1> velocity = {0.0, 0.0};
};

#endif // CTP_SWAP_STATE_HPP