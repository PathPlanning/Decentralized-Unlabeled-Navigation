#ifndef CTP_SWAP_ACTION_HPP
#define CTP_SWAP_ACTION_HPP

#include <xtensor/xtensor.hpp>

struct Action {
	xt::xtensor<double, 1> velocity = {0.0, 0.0};
	xt::xtensor<double, 1> goal = {0.0, 0.0};
};

#endif // CTP_SWAP_ACTION_HPP
