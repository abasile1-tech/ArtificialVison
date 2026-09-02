#include "maze_viewer.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/rect2.hpp>

#include <algorithm>
#include <cmath>
#include <queue>

namespace godot {

MazeViewer::MazeViewer() : maze_(width_ * height_, '#'), belief_(width_ * height_, 0) {}

void MazeViewer::_bind_methods() {}

void MazeViewer::_ready() {
    reset();
    set_process(true);
}

void MazeViewer::reset() {
    std::fill(maze_.begin(), maze_.end(), '#');
    std::fill(belief_.begin(), belief_.end(), 0);
    rng_.seed(42);
    robot_ = {1, 1};
    heading_ = 1;
    generate_maze();
    route_ = find_path(robot_, goal_);
    route_index_ = 0;
    elapsed_ = 0.0;
    observe();
}

void MazeViewer::generate_maze() {
    maze_[index({1, 1})] = '.';
    std::vector<Point> stack{{1, 1}};
    while (!stack.empty()) {
        Point current = stack.back();
        std::vector<Point> candidates;
        for (Point direction : directions_) {
            Point next{current.x + direction.x * 2, current.y + direction.y * 2};
            if (next.x > 0 && next.x < width_ - 1 && next.y > 0 && next.y < height_ - 1 && maze_at(next) == '#') candidates.push_back(next);
        }
        if (candidates.empty()) { stack.pop_back(); continue; }
        Point next = candidates[std::uniform_int_distribution<size_t>(0, candidates.size() - 1)(rng_)];
        maze_[index({(current.x + next.x) / 2, (current.y + next.y) / 2})] = '.';
        maze_[index(next)] = '.';
        stack.push_back(next);
    }
}

void MazeViewer::_process(double delta) {
    elapsed_ += delta;
    if (elapsed_ < 0.11 || robot_ == goal_ || route_index_ + 1 >= route_.size()) return;
    elapsed_ = 0.0;
    Point next = route_[++route_index_];
    for (int i = 0; i < 4; ++i) {
        if (robot_.x + directions_[i].x == next.x && robot_.y + directions_[i].y == next.y) heading_ = i;
    }
    robot_ = next;
    observe();
    queue_redraw();
}

MazeViewer::Reading MazeViewer::range_sensor(int sensor_heading) {
    constexpr int max_range = 8;
    int distance = 0;
    bool hit_wall = false;
    for (int step = 1; step <= max_range; ++step) {
        Point probe{robot_.x + directions_[sensor_heading].x * step, robot_.y + directions_[sensor_heading].y * step};
        if (is_wall(probe)) { distance = step - 1; hit_wall = true; break; }
        distance = step;
    }
    return {std::max(0.0, distance + noise_(rng_)), hit_wall};
}

void MazeViewer::observe() {
    update_belief(robot_, -4);
    const std::array<int, 3> headings{(heading_ + 3) % 4, heading_, (heading_ + 1) % 4};
    for (size_t i = 0; i < readings_.size(); ++i) {
        readings_[i] = range_sensor(headings[i]);
        integrate_beam(headings[i], readings_[i]);
    }
}

void MazeViewer::integrate_beam(int sensor_heading, const Reading& reading) {
    const int free_cells = std::clamp(static_cast<int>(std::lround(reading.cells)), 0, 8);
    for (int step = 0; step <= free_cells; ++step) update_belief({robot_.x + directions_[sensor_heading].x * step, robot_.y + directions_[sensor_heading].y * step}, -1);
    if (reading.hit_wall && free_cells < 8) update_belief({robot_.x + directions_[sensor_heading].x * (free_cells + 1), robot_.y + directions_[sensor_heading].y * (free_cells + 1)}, 3);
}

void MazeViewer::update_belief(Point point, int amount) {
    if (in_bounds(point)) belief_[index(point)] = std::clamp(belief_[index(point)] + amount, -8, 8);
}

std::vector<MazeViewer::Point> MazeViewer::find_path(Point start, Point destination) const {
    struct Node { Point point; int cost; int estimate; };
    auto compare = [](const Node& a, const Node& b) { return a.estimate > b.estimate; };
    std::priority_queue<Node, std::vector<Node>, decltype(compare)> frontier(compare);
    std::vector<int> cost(width_ * height_, 1'000'000);
    std::vector<Point> parent(width_ * height_, {-1, -1});
    auto estimate = [destination](Point p) { return std::abs(p.x - destination.x) + std::abs(p.y - destination.y); };
    frontier.push({start, 0, estimate(start)});
    cost[index(start)] = 0;
    while (!frontier.empty()) {
        Node current = frontier.top(); frontier.pop();
        if (current.point == destination) break;
        for (Point direction : directions_) {
            Point next{current.point.x + direction.x, current.point.y + direction.y};
            if (is_wall(next)) continue;
            const int next_cost = current.cost + 1;
            if (next_cost >= cost[index(next)]) continue;
            cost[index(next)] = next_cost;
            parent[index(next)] = current.point;
            frontier.push({next, next_cost, next_cost + estimate(next)});
        }
    }
    std::vector<Point> path;
    for (Point point = destination; !(point == start); point = parent[index(point)]) {
        if (point.x < 0) return {};
        path.push_back(point);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

void MazeViewer::_draw() {
    const Vector2 world_origin{28, 86};
    const Vector2 map_origin{660, 86};
    const Color wall("26313d"), floor("dbe8f3"), route("a78bfa"), goal("36c995"), unknown("17212b"), occupied("d15454"), robot("3989ff");
    for (int y = 0; y < height_; ++y) for (int x = 0; x < width_; ++x) {
        Point point{x, y};
        Color world_color = maze_at(point) == '#' ? wall : floor;
        if (std::find(route_.begin(), route_.end(), point) != route_.end()) world_color = route;
        if (point == goal_) world_color = goal;
        draw_rect(Rect2(world_origin + Vector2(x * tile_, y * tile_), Vector2(tile_ - 1, tile_ - 1)), world_color);
        Color map_color = unknown;
        if (belief_at(point) == '.') map_color = floor;
        if (belief_at(point) == '#') map_color = occupied;
        draw_rect(Rect2(map_origin + Vector2(x * tile_, y * tile_), Vector2(tile_ - 1, tile_ - 1)), map_color);
    }
    for (Vector2 origin : {world_origin, map_origin}) {
        const Vector2 robot_center = center(robot_, origin);
        draw_circle(robot_center, tile_ * 0.36f, robot);
        draw_line(robot_center, robot_center + Vector2(directions_[heading_].x, directions_[heading_].y) * tile_ * 0.55f, Color(1, 1, 1), 3.0f);
    }
    const std::array<Color, 3> sensor_colors{Color("5cc8ff"), Color("ffd166"), Color("ff7b7b")};
    const std::array<int, 3> headings{(heading_ + 3) % 4, heading_, (heading_ + 1) % 4};
    const Vector2 start = center(robot_, world_origin);
    for (size_t i = 0; i < readings_.size(); ++i) {
        const Vector2 end = start + Vector2(directions_[headings[i]].x, directions_[headings[i]].y) * static_cast<float>(readings_[i].cells * tile_);
        draw_line(start, end, sensor_colors[i], 2.0f);
    }
}

bool MazeViewer::in_bounds(Point point) const { return point.x >= 0 && point.x < width_ && point.y >= 0 && point.y < height_; }
bool MazeViewer::is_wall(Point point) const { return !in_bounds(point) || maze_at(point) == '#'; }
int MazeViewer::index(Point point) const { return point.y * width_ + point.x; }
char MazeViewer::maze_at(Point point) const { return maze_[index(point)]; }
char MazeViewer::belief_at(Point point) const { return belief_[index(point)] >= 2 ? '#' : belief_[index(point)] <= -2 ? '.' : '?'; }
Vector2 MazeViewer::center(Point point, Vector2 origin) const { return origin + Vector2(point.x * tile_ + tile_ / 2.0f, point.y * tile_ + tile_ / 2.0f); }

} // namespace godot
