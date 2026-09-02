#pragma once

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/variant/vector2i.hpp>

#include <array>
#include <random>
#include <vector>

namespace godot {

class MazeViewer : public Node2D {
    GDCLASS(MazeViewer, Node2D)

public:
    MazeViewer();
    void _ready() override;
    void _process(double delta) override;
    void _draw() override;

protected:
    static void _bind_methods();

private:
    struct Point {
        int x{};
        int y{};
        bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    };
    struct Reading { double cells{}; bool hit_wall{}; };

    static constexpr int width_ = 31;
    static constexpr int height_ = 21;
    static constexpr int tile_ = 18;
    static constexpr std::array<Point, 4> directions_{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

    std::vector<char> maze_;
    std::vector<int> belief_;
    std::vector<Point> route_;
    std::array<Reading, 3> readings_{};
    Point robot_{1, 1};
    Point goal_{width_ - 2, height_ - 2};
    int heading_ = 1;
    size_t route_index_ = 0;
    double elapsed_ = 0.0;
    std::mt19937 rng_{42};
    std::normal_distribution<double> noise_{0.0, 0.18};

    void reset();
    void generate_maze();
    void observe();
    void integrate_beam(int sensor_heading, const Reading& reading);
    void update_belief(Point point, int amount);
    Reading range_sensor(int sensor_heading);
    std::vector<Point> find_path(Point start, Point destination) const;
    bool in_bounds(Point point) const;
    bool is_wall(Point point) const;
    int index(Point point) const;
    char maze_at(Point point) const;
    char belief_at(Point point) const;
    Vector2 center(Point point, Vector2 origin) const;
};

} // namespace godot
