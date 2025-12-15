#ifndef CTP_SWAP_ORCA_FOLLOWER_HPP
#define CTP_SWAP_ORCA_FOLLOWER_HPP

#include <../libs/rvo2/src/RVOSimulator.h>
#include <../libs/rvo2/src/Vector2.h>
#include <cstdint>
#include <memory>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xrandom.hpp>
#include<list>

#include "action.hpp"
#include "follower.hpp"
#include "state.hpp"

struct ORCAParams : public BaseAlgParams {
    // TODO readable
    float timestep = 0.1;
    float sight_radius = 3.0;
    size_t agents_max_num = 10;
    float time_boundary = 3.0;
    float time_boundary_obst = 3.0;
    float r_eps = 0.15;
    float max_speed = 1.0;
};


class ORCAFollower : public Follower {
  public:
    ORCAFollower(HolonomicAgentParams ag_params,
                 std::shared_ptr<BaseAlgParams> follower_params,
                 const std::vector<std::vector<xt::xtensor<double, 1>>>& obstacles);

    void updateNeighborsInfo(std::vector<State> neighbors_states,
                             std::vector<float> neighbors_sizes) override;

    Action findAction(const State &state,
                      const xt::xtensor<double, 1> &waypoint) override;

  protected:
    void initialize();

    std::shared_ptr<RVO::RVOSimulator> rvo_sim;
    std::shared_ptr<ORCAParams> orca_params;
    size_t a_id;
};

#endif // CTP_SWAP_ORCA_FOLLOWER_HPP