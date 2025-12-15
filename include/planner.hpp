#ifndef CTP_SWAP_PLANNER_HPP
#define CTP_SWAP_PLANNER_HPP

#include <cstdint>
#include <utility>
#include <xtensor/xtensor.hpp>

#include "environment_description.hpp"
#include "params.hpp"

class Planner {

  public:
    virtual ~Planner() = default;

    Planner() = delete;
    Planner(const HolonomicAgentParams &ag_params, const std::shared_ptr<BaseAlgParams> &planner_params,
            const std::vector<xt::xtensor<double, 1>> &goals, const std::shared_ptr<EnvironmentDescription> &env)
        : ag_params(ag_params), planner_params(planner_params), goals(goals), env(env) {};

    virtual xt::xtensor<double, 1> findNext(const xt::xtensor<double, 1> &pos,
                                            const xt::xtensor<double, 1> &goal) = 0;
    virtual double findLength(const xt::xtensor<double, 1> &pos,
                              const xt::xtensor<double, 1> &goal) = 0;

    virtual std::vector<double> findAllLengths(const xt::xtensor<double, 1> &pos) = 0;

  protected:
    HolonomicAgentParams ag_params;
    std::shared_ptr<BaseAlgParams> planner_params;
    std::vector<xt::xtensor<double, 1>> goals;
    std::shared_ptr<EnvironmentDescription> env;
};

#endif // CTP_SWAP_PLANNER_HPP