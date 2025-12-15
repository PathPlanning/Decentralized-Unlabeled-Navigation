#ifndef CTP_SWAP_MESSAGE_HPP
#define CTP_SWAP_MESSAGE_HPP

#include <cstdint>
#include <xtensor/xtensor.hpp>
#include "state.hpp"

struct Message {

	int64_t id;
	int64_t priority;
	State state;
	double size;
	xt::xtensor<double, 1> goal = {0, 0}; 
	size_t goal_id;
	std::vector<int64_t> goals_priority_table;
	// std::vector<double> goals_lengths;
};

#endif // CTP_SWAP_MESSAGE_HPP