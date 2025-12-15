#ifndef CTP_SWAP_VISIBILITY_GRAPH_HPP
#define CTP_SWAP_VISIBILITY_GRAPH_HPP

#include "environment_description.hpp"

#include "trivis/trivis.h"

#include <xtensor/xtensor.hpp>
#include <vector>


using XTPoint = xt::xtensor<double, 1>;
using XTPolygon = std::vector<XTPoint>;


class VisibilityGraph : public EnvironmentDescription {
public:
    VisibilityGraph() = default;
    VisibilityGraph(const std::vector<XTPolygon> &obstacles, const std::vector<XTPoint> &goals, double offset);
    void initialize();

    [[nodiscard]] std::vector<int64_t> getVisible(const XTPoint& point) const;
    [[nodiscard]] std::vector<int64_t> findNeighbors(const int64_t& v_id) const;
    [[nodiscard]] XTPoint getPoint(const int64_t &v_id) const;
    [[nodiscard]] int getPointsCount() const {return point_point_graph.size();}

private:
    void makeOffset();
    void construct();

    std::vector<XTPolygon> obstacles;
    std::vector<XTPoint> goals;
    std::vector<XTPoint> points;
    double offset{};

    trivis::Trivis vis;
    std::vector<std::vector<int>> point_point_graph;
    std::vector<std::optional<trivis::Trivis::PointLocationResult>> input_plrs;
    std::vector<trivis::geom::FPoint> input_points;
    std::pair<double, double> up_right;
    std::pair<double, double> bottom_left;

};


#endif //CTP_SWAP_VISIBILITY_GRAPH_HPP
