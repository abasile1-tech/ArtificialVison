// A dependency-free first milestone for a maze-navigation robot.
// It has a generated world, noisy range sensors, a symbolic forward camera,
// A* navigation, and a terminal renderer.

#include "ArtificialVison.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Point { int x{}; int y{}; friend bool operator==(const Point&, const Point&) = default; };
constexpr int kWidth = 31, kHeight = 21;
constexpr std::array<Point, 4> kDirections{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

class Maze {
public:
    Maze() : cells_(kWidth * kHeight, '#') { generate(); }
    [[nodiscard]] bool inBounds(Point p) const { return p.x >= 0 && p.x < kWidth && p.y >= 0 && p.y < kHeight; }
    [[nodiscard]] bool isWall(Point p) const { return !inBounds(p) || at(p) == '#'; }
    [[nodiscard]] char at(Point p) const { return cells_[static_cast<size_t>(p.y * kWidth + p.x)]; }
    void set(Point p, char value) { cells_[static_cast<size_t>(p.y * kWidth + p.x)] = value; }
private:
    void generate() {
        std::mt19937 rng(42); // Fixed seed makes the introductory scenario reproducible.
        std::vector<Point> stack{{1, 1}};
        set({1, 1}, '.');
        while (!stack.empty()) {
            Point current = stack.back();
            std::vector<Point> candidates;
            for (Point direction : kDirections) {
                Point next{current.x + direction.x * 2, current.y + direction.y * 2};
                if (next.x > 0 && next.x < kWidth - 1 && next.y > 0 && next.y < kHeight - 1 && at(next) == '#') candidates.push_back(next);
            }
            if (candidates.empty()) { stack.pop_back(); continue; }
            Point next = candidates[std::uniform_int_distribution<size_t>(0, candidates.size() - 1)(rng)];
            set({(current.x + next.x) / 2, (current.y + next.y) / 2}, '.');
            set(next, '.');
            stack.push_back(next);
        }
    }
    std::vector<char> cells_;
};

struct Robot { Point position{1, 1}; int heading{1}; }; // East initially.

class SensorSuite {
public:
    explicit SensorSuite(const Maze& maze) : maze_(maze) {}
    [[nodiscard]] std::array<double, 3> scan(const Robot& robot) {
        return {range(robot.position, (robot.heading + 3) % 4), range(robot.position, robot.heading), range(robot.position, (robot.heading + 1) % 4)};
    }
private:
    [[nodiscard]] double range(Point origin, int heading) {
        constexpr int maxRange = 8;
        int distance = 0;
        for (int step = 1; step <= maxRange; ++step) {
            Point probe{origin.x + kDirections[heading].x * step, origin.y + kDirections[heading].y * step};
            if (maze_.isWall(probe)) { distance = step - 1; break; }
            distance = step;
        }
        return std::max(0.0, distance + noise_(rng_));
    }
    const Maze& maze_;
    std::mt19937 rng_{7};
    std::normal_distribution<double> noise_{0.0, 0.18};
};

[[nodiscard]] std::vector<Point> findPath(const Maze& maze, Point start, Point goal) {
    struct Node { Point point; int cost; int estimate; };
    auto compare = [](const Node& a, const Node& b) { return a.estimate > b.estimate; };
    std::priority_queue<Node, std::vector<Node>, decltype(compare)> frontier(compare);
    std::vector<int> cost(kWidth * kHeight, 1'000'000);
    std::vector<Point> parent(kWidth * kHeight, {-1, -1});
    auto index = [](Point p) { return p.y * kWidth + p.x; };
    auto heuristic = [goal](Point p) { return std::abs(p.x - goal.x) + std::abs(p.y - goal.y); };
    frontier.push({start, 0, heuristic(start)});
    cost[static_cast<size_t>(index(start))] = 0;
    while (!frontier.empty()) {
        Node current = frontier.top(); frontier.pop();
        if (current.point == goal) break;
        for (Point direction : kDirections) {
            Point next{current.point.x + direction.x, current.point.y + direction.y};
            if (maze.isWall(next)) continue;
            int nextCost = current.cost + 1;
            if (nextCost >= cost[static_cast<size_t>(index(next))]) continue;
            cost[static_cast<size_t>(index(next))] = nextCost;
            parent[static_cast<size_t>(index(next))] = current.point;
            frontier.push({next, nextCost, nextCost + heuristic(next)});
        }
    }
    std::vector<Point> path;
    for (Point p = goal; !(p == start); p = parent[static_cast<size_t>(index(p))]) {
        if (p.x < 0) return {};
        path.push_back(p);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

[[nodiscard]] char cameraPixel(const Maze& maze, const Robot& robot, int forward, int side, Point goal) {
    Point view{robot.position.x + kDirections[robot.heading].x * forward + kDirections[(robot.heading + 1) % 4].x * side,
               robot.position.y + kDirections[robot.heading].y * forward + kDirections[(robot.heading + 1) % 4].y * side};
    if (view == goal) return 'G';
    return maze.isWall(view) ? '#' : '.';
}

void render(const Maze& maze, const Robot& robot, Point goal, const std::vector<Point>& route, const std::array<double, 3>& readings, int frame) {
    std::cout << "\x1B[2J\x1B[H"; // Clear an ANSI-compatible terminal.
    std::cout << "Artificial Vision Maze Robot | frame " << frame << " | goal: " << goal.x << ',' << goal.y << "\n\n";
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            Point p{x, y}; char pixel = maze.at(p);
            if (std::find(route.begin(), route.end(), p) != route.end() && pixel != '#') pixel = '+';
            if (p == goal) pixel = 'G';
            if (p == robot.position) pixel = ">v<^"[robot.heading];
            std::cout << pixel;
        }
        std::cout << '\n';
    }
    std::cout << "\nNoisy range sensors (cells): left " << readings[0] << " | front " << readings[1] << " | right " << readings[2] << "\n";
    std::cout << "Forward camera (# wall, . floor, G goal):\n";
    for (int forward = 5; forward >= 1; --forward) {
        for (int side = -3; side <= 3; ++side) std::cout << cameraPixel(maze, robot, forward, side, goal);
        std::cout << '\n';
    }
    std::cout << "\nLegend: # wall, + A* route, >v<^ robot heading, G goal. Ctrl+C exits.\n";
}

[[nodiscard]] int optionValue(int argc, char* argv[], const std::string& option, int fallback) {
    for (int i = 1; i + 1 < argc; ++i) if (option == argv[i]) return std::max(0, std::atoi(argv[i + 1]));
    return fallback;
}

} // namespace

int main(int argc, char* argv[]) {
    int delayMs = optionValue(argc, argv, "--delay", 110);
    int requestedSteps = optionValue(argc, argv, "--steps", 10'000);
    Maze maze; Robot robot; Point goal{kWidth - 2, kHeight - 2};
    std::vector<Point> path = findPath(maze, robot.position, goal);
    SensorSuite sensors(maze);
    if (path.empty()) { std::cerr << "No route to the goal.\n"; return 1; }
    size_t pathIndex = 0;
    for (int frame = 0; frame < requestedSteps; ++frame) {
        auto readings = sensors.scan(robot);
        render(maze, robot, goal, {path.begin() + static_cast<std::ptrdiff_t>(pathIndex), path.end()}, readings, frame);
        if (robot.position == goal) { std::cout << "Goal reached after " << frame << " motion steps.\n"; return 0; }
        Point next = path[++pathIndex];
        for (int heading = 0; heading < 4; ++heading) if (robot.position.x + kDirections[heading].x == next.x && robot.position.y + kDirections[heading].y == next.y) robot.heading = heading;
        robot.position = next;
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
    }
    std::cout << "Stopped after requested step limit. Use --steps 10000 for a full run.\n";
}
