#ifndef CTP_SWAP_VISIBILITY_PLANNER_HPP
#define CTP_SWAP_VISIBILITY_PLANNER_HPP

#include <memory>
#include <cassert>
#include <cstdint>
#include <unordered_map>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xtensor.hpp>
#include <iostream>
#include "planner.hpp"
#include "visibility_graph.hpp"
#include "dijkstra.hpp"


struct PointHash {
    size_t operator()(const xt::xtensor<double, 1>& point) const {
        auto h1 = std::hash<double>{}(point(0));
        auto h2 = std::hash<double>{}(point(1));
        return h1 ^ (h2 << 1); // Combine hashes
    }
};

struct PointEqual {
    bool operator()(const xt::xtensor<double, 1>& p1, const xt::xtensor<double, 1>& p2) const {
        return p1(0) == p2(0) && p1(1) == p2(1);
    }
};




struct VisibilityPlannerParams : public BaseAlgParams {
};


class VisibilityPlanner : public Planner {
public:
    VisibilityPlanner(const HolonomicAgentParams &ag_params, const std::shared_ptr<BaseAlgParams> &planner_params,
                      const std::vector<xt::xtensor<double, 1> > &goals, const std::shared_ptr<EnvironmentDescription> &env);

    xt::xtensor<double, 1> findNext(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) override;

    double findLength(const xt::xtensor<double, 1> &pos, const xt::xtensor<double, 1> &goal) override;

    std::vector<double> findAllLengths(const xt::xtensor<double, 1> &pos) override;

protected:
    void initialize();

    std::shared_ptr<VisibilityGraph> graph;
    Dijkstra search;
    std::vector<std::unordered_map<int64_t, std::shared_ptr<Node<int64_t>>, std::hash<int64_t>>> path_table;
    std::unordered_map<xt::xtensor<double, 1>, size_t, PointHash, PointEqual> point_index;
    std::unordered_set<int64_t> goals_set;
};


#endif // CTP_SWAP_VISIBILITY_PLANNER_HPP
