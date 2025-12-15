#ifndef CTP_SWAP_AGENT_HPP
#define CTP_SWAP_AGENT_HPP

#include <memory>
#include <vector>

#include "action.hpp"
#include "message.hpp"
#include "params.hpp"
#include "state.hpp"
#include "planner.hpp"
#include "follower.hpp"

class Agent {

  public:
	Agent() = delete;
	Agent(int64_t a_id, const State &initial_state,
		  const std::vector<xt::xtensor<double, 1>> &goals,
		  const HolonomicAgentParams &ag_params,
		  std::shared_ptr<BaseAlgParams> alg_params,
          std::shared_ptr<Planner> planner,
          std::shared_ptr<Follower> follower)
		: a_id(a_id), state(initial_state), goals(goals), ag_params(ag_params),
		  alg_params(alg_params), planner(planner), follower(follower) {};

	virtual bool initialize() = 0;
	virtual void
	updateNeighborsInfo(const std::vector<Message> &neighbors_info) = 0;
	virtual Action computeAction() = 0;
	virtual void updateStateInfo(const State &new_state) = 0;
	virtual Message sendMessage() = 0;

  protected:
	int64_t a_id;
	State state;
	std::vector<xt::xtensor<double, 1>> goals;
	HolonomicAgentParams ag_params;
	std::shared_ptr<BaseAlgParams> alg_params;
    std::shared_ptr<Planner> planner;
    std::shared_ptr<Follower> follower;
};

#endif // CTP_SWAP_AGENT_HPP
