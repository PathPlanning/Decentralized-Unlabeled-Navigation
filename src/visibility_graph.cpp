#include "visibility_graph.hpp"


XTPoint perpendicular(const XTPoint &v) {
    return XTPoint({-v(1), v(0)});
}


XTPoint normalize(const XTPoint &v) {
    double length = std::sqrt(xt::sum(v * v)());
    return v / length;
}

XTPolygon inflatePolygon(const XTPolygon &polygon, double offset) {
    XTPolygon inflated;
    size_t n = polygon.size();

    std::vector<XTPoint> normals(n);

    // Calculate normals for each edge
    for (size_t i = 0; i < n; ++i) {
        XTPoint p1 = polygon[i];
        XTPoint p2 = polygon[(i + 1) % n];
        XTPoint edge = p1 - p2;
        normals[i] = normalize(perpendicular(edge)) * offset;
    }

    // Calculate inflated vertices by averaging adjacent normals
    for (size_t i = 0; i < n; ++i) {
        XTPoint p = polygon[i];
        XTPoint normal_prev = normals[(i + n - 1) % n];
        XTPoint normal_current = normals[i];

        XTPoint inflated_point = p + (normal_prev + normal_current) / 2;
        inflated.push_back(inflated_point);
    }

    return inflated;
}


VisibilityGraph::VisibilityGraph(const std::vector<XTPolygon> &obstacles,
                                 const std::vector<XTPoint> &goals, double offset) : obstacles(
        obstacles), goals(goals), offset(offset) {
}

void VisibilityGraph::construct() { {
        // Scope for the environment.
        trivis::geom::PolyMap environment;
        trivis::geom::FPolygons holes;
        trivis::geom::FPolygon curr_polygon;
        for (size_t obst_id = 0; obst_id < obstacles.size() - 1; ++obst_id) {
            curr_polygon.clear();

            for (auto &point: obstacles[obst_id]) {
                curr_polygon.emplace_back(point[0], point[1]);
            }

            if (OrientationCounterClockwise(curr_polygon)) {
                ChangeOrientation(curr_polygon);
            }
            holes.push_back(curr_polygon);
        }
        double x_min = -1.0, x_max = up_right.first+2*offset, y_min = -1.0, y_max = up_right.second+2*offset;

        curr_polygon.clear();
        curr_polygon.emplace_back(x_min, y_min);
        curr_polygon.emplace_back(x_min, y_max);
        curr_polygon.emplace_back(x_max, y_max);
        curr_polygon.emplace_back(x_max, y_min);

        if (not OrientationCounterClockwise(curr_polygon)) {
            ChangeOrientation(curr_polygon);
        }
        const auto border = curr_polygon;
        auto map = trivis::geom::PolyMap(border, holes);
        vis = trivis::Trivis{std::move(map)};
    }
    input_points.reserve(points.size());

    for (auto point : points) {
        input_points.emplace_back(point[0], point[1]);
    }

    input_plrs.reserve(input_points.size());
    for (const auto &p: input_points) {
        input_plrs.push_back(vis.LocatePoint(p));
    }
    constexpr std::optional<double> range = std::nullopt;

    point_point_graph = vis.PointPointVisibilityGraph(input_points, input_plrs, range);
}

void VisibilityGraph::makeOffset() {
    for (size_t i = 0; i < obstacles.size() - 1; ++i) {

        auto inflated = inflatePolygon(obstacles[i], offset + 0.07);
        for (auto &point: inflated) {
            if (point[0] < bottom_left.first or point[0] > up_right.first or point[1] < bottom_left.second or point[1] > up_right.second) {
                continue;
            }
            points.push_back(point);
        }
    }

    std::vector<XTPolygon> new_obstacles;
    for (size_t i = 0; i < obstacles.size(); ++i) {
        auto inflated = inflatePolygon(obstacles[i], offset);
        new_obstacles.emplace_back(inflated);
    }
    obstacles = new_obstacles;
}

std::vector<int64_t> VisibilityGraph::findNeighbors(const int64_t &v_id) const {
    std::vector<int64_t> result;
    result.insert(result.end(), point_point_graph[v_id].begin(), point_point_graph[v_id].end());
    return result;
}

XTPoint VisibilityGraph::getPoint(const int64_t &v_id) const {
    return points[v_id];
}

void VisibilityGraph::initialize() {
    for (auto &point: obstacles.back()) {
        const auto x = point[0];
        const auto y = point[1];
        if (x > up_right.first) {
            up_right.first = x;
        }
        if (y > up_right.second) {
            up_right.second = y;
        }

        if (x < bottom_left.first) {
            bottom_left.first = x;
        }
        if (y < bottom_left.second) {
            bottom_left.second = y;
        }
    }
    points.insert(points.end(), goals.begin(), goals.end());
    makeOffset();
    construct();
}

std::vector<int64_t> VisibilityGraph::getVisible(const XTPoint &point) const {
    std::vector<int64_t> visible;
    const trivis::geom::FPoint query = {point[0], point[1]};
    if (const auto plr = vis.LocatePoint(query); plr.has_value()) {
        const auto visible_points = vis.VisiblePoints(query, plr.value(), input_points, input_plrs, std::nullopt);
        for (const auto &point_id: visible_points) {
            visible.push_back(point_id);
        }
    }
    return visible;
}






