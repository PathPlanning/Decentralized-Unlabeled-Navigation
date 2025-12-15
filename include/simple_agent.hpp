#ifndef CTP_SWAP_SIMPLE_AGENT_HPP
#define CTP_SWAP_SIMPLE_AGENT_HPP

#include <vector>

#include "agent.hpp"
#include "params.hpp"
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xtensor.hpp>

struct SimpleNavParams : public BaseAlgParams {
};


class SimpleAgent : public Agent {

  public:
	SimpleAgent(uint64_t a_id, const State &initial_state,
				const std::vector<xt::xtensor<double, 1>> &goals,
				const HolonomicAgentParams &ag_params,
				std::shared_ptr<BaseAlgParams> alg_params,
				std::shared_ptr<Planner> planner,
				std::shared_ptr<Follower> follower)
		: Agent(a_id, initial_state, goals, ag_params, alg_params, nullptr,
				nullptr) {};
	bool initialize() override;
	Message sendMessage() override;
	void
	updateNeighborsInfo(const std::vector<Message> &neighbors_info) override;
	void updateStateInfo(const State &new_state) override;
	Action computeAction() override;

  protected:
	xt::xtensor<double, 1> goal;
};

#endif // CTP_SWAP_SIMPLE_AGENT_HPP