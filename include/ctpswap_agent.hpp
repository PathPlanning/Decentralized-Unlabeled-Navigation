#ifndef CTP_SWAP_AMAPF_AGENT_HPP
#define CTP_SWAP_AMAPF_AGENT_HPP

#include <limits>
#include <vector>
#include <list>

#include "agent.hpp"
#include "params.hpp"
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xrandom.hpp>

struct CTPSWAPParams : public BaseAlgParams {
    bool amapf = false;
};

class CTPSWAPAgent : public Agent {

  public:
    CTPSWAPAgent(int64_t a_id, const State &initial_state,
                 const std::vector<xt::xtensor<double, 1>> &goals,
                 const HolonomicAgentParams &ag_params,
                 std::shared_ptr<BaseAlgParams> alg_params,
                 std::shared_ptr<Planner> planner,
                 std::shared_ptr<Follower> follower)
        : Agent(a_id, initial_state, goals, ag_params, alg_params, planner,
                follower) {
        step = 0;
        priority = a_id;
        ctp_swap_params = std::static_pointer_cast<CTPSWAPParams>(alg_params);
    };
    bool initialize() override;
    Message sendMessage() override;
    void
    updateNeighborsInfo(const std::vector<Message> &neighbors_info) override;
    void updateStateInfo(const State &new_state) override;
    Action computeAction() override;

  protected:
    std::shared_ptr<CTPSWAPParams> ctp_swap_params;
    size_t step;
    void updateGoals();
    bool reachedAgentLieOnPath(const xt::xtensor<double, 1> &pos_a1,
                               const xt::xtensor<double, 1> &pos_a2,
                               const xt::xtensor<double, 1> &goal_a1,
                               const xt::xtensor<double, 1> &goal_a2) const;

    void computeAvg(const Action &action);

    xt::xtensor<double, 1> goal;
    size_t goal_id;
    int64_t priority;
    std::vector<int64_t> goals_priority_table;
    std::unordered_map<int64_t, size_t> curent_goal_assignment;
    std::unordered_map<int64_t, int64_t> curent_priority_assignment;
    std::unordered_map<int64_t, xt::xtensor<double, 1>> curent_positions;
    // std::unordered_map<int64_t, std::vector<double>> current_neighbors_length;

    double sum_speed = 0;
    std::list<double> buffer;
};

#endif // CTP_SWAP_AMAPF_AGENT_HPP