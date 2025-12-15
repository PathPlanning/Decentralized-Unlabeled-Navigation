#ifndef CTP_SWAP_DIRECT_PLANNER_HPP
#define CTP_SWAP_DIRECT_PLANNER_HPP

#include <cstdint>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xtensor.hpp>

#include "planner.hpp"

struct DirectPlannerParams : public BaseAlgParams {
};

class DirectPlanner : public Planner {

public:
    DirectPlanner(const HolonomicAgentParams &ag_params, const std::shared_ptr<BaseAlgParams> &planner_params,
                  const std::vector<xt::xtensor<double, 1>> &goals, const std::shared_ptr<EnvironmentDescription> &env)
            : Planner(ag_params, planner_params, goals, env) {};

    xt::xtensor<double, 1> findNext(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) override;

    double findLength(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) override;

    std::vector<double> findAllLengths(const xt::xtensor<double, 1> &pos) override;
};

#endif // CTP_SWAP_DIRECT_PLANNER_HPP