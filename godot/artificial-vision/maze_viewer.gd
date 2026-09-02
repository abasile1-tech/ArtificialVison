extends Node2D

# First visual milestone. This intentionally mirrors the terminal model while
# the C++ classes remain independent and ready to become a GDExtension later.
const GRID_WIDTH := 31
const GRID_HEIGHT := 21
const TILE := 18
const LEFT_ORIGIN := Vector2(28, 86)
const MAP_ORIGIN := Vector2(660, 86)
const DIRECTIONS := [Vector2i(0, -1), Vector2i(1, 0), Vector2i(0, 1), Vector2i(-1, 0)]
const SENSOR_COLORS := [Color("5cc8ff"), Color("ffd166"), Color("ff7b7b")]

var maze := []
var belief := []
var robot := Vector2i(1, 1)
var goal := Vector2i(GRID_WIDTH - 2, GRID_HEIGHT - 2)
var heading := 1
var route := []
var route_index := 0
var readings := []
var elapsed := 0.0
var paused := false
var frame := 0
var rng := RandomNumberGenerator.new()

func _ready() -> void:
    rng.seed = 42
    reset_simulation()

func reset_simulation() -> void:
    maze.clear()
    belief.clear()
    for y in GRID_HEIGHT:
        var maze_row := []
        var belief_row := []
        for x in GRID_WIDTH:
            maze_row.append("#")
            belief_row.append(0) # 0 unknown; negative free; positive occupied.
        maze.append(maze_row)
        belief.append(belief_row)
    generate_maze()
    robot = Vector2i(1, 1)
    heading = 1
    goal = Vector2i(GRID_WIDTH - 2, GRID_HEIGHT - 2)
    route = find_path(robot, goal)
    route_index = 0
    elapsed = 0.0
    frame = 0
    observe()
    queue_redraw()

func _process(delta: float) -> void:
    if not paused:
        elapsed += delta
        if elapsed >= 0.11:
            elapsed = 0.0
            advance_robot()
    queue_redraw()

func _unhandled_key_input(event: InputEvent) -> void:
    if event.is_pressed() and event.keycode == KEY_SPACE:
        paused = not paused
    elif event.is_pressed() and event.keycode == KEY_R:
        reset_simulation()

func generate_maze() -> void:
    set_maze_cell(Vector2i(1, 1), ".")
    var stack := [Vector2i(1, 1)]
    while not stack.is_empty():
        var current: Vector2i = stack.back()
        var candidates := []
        for direction in DIRECTIONS:
            var next: Vector2i = current + direction * 2
            if next.x > 0 and next.x < GRID_WIDTH - 1 and next.y > 0 and next.y < GRID_HEIGHT - 1 and maze_cell(next) == "#":
                candidates.append(next)
        if candidates.is_empty():
            stack.pop_back()
            continue
        var next: Vector2i = candidates[rng.randi_range(0, candidates.size() - 1)]
        set_maze_cell((current + next) / 2, ".")
        set_maze_cell(next, ".")
        stack.append(next)

func find_path(start: Vector2i, destination: Vector2i) -> Array:
    var open := [{"point": start, "cost": 0, "estimate": manhattan(start, destination)}]
    var costs := {start: 0}
    var parents := {}
    while not open.is_empty():
        var best_index := 0
        for i in range(1, open.size()):
            if open[i].estimate < open[best_index].estimate:
                best_index = i
        var current: Dictionary = open.pop_at(best_index)
        var current_point: Vector2i = current.point
        if current_point == destination:
            break
        for direction in DIRECTIONS:
            var next: Vector2i = current_point + direction
            if is_wall(next):
                continue
            var next_cost: int = current.cost + 1
            if not costs.has(next) or next_cost < costs[next]:
                costs[next] = next_cost
                parents[next] = current_point
                open.append({"point": next, "cost": next_cost, "estimate": next_cost + manhattan(next, destination)})
    var result := []
    var point := destination
    while point != start:
        if not parents.has(point):
            return []
        result.append(point)
        point = parents[point]
    result.append(start)
    result.reverse()
    return result

func advance_robot() -> void:
    if robot == goal or route_index + 1 >= route.size():
        paused = true
        return
    var next: Vector2i = route[route_index + 1]
    for i in DIRECTIONS.size():
        if robot + DIRECTIONS[i] == next:
            heading = i
            break
    robot = next
    route_index += 1
    frame += 1
    observe()

func observe() -> void:
    readings.clear()
    update_belief(robot, -4)
    for sensor_heading in [(heading + 3) % 4, heading, (heading + 1) % 4]:
        var reading := range_sensor(sensor_heading)
        readings.append(reading)
        integrate_beam(sensor_heading, reading)

func range_sensor(sensor_heading: int) -> Dictionary:
    const MAX_RANGE := 8
    var distance := 0
    var hit_wall := false
    for step in range(1, MAX_RANGE + 1):
        if is_wall(robot + DIRECTIONS[sensor_heading] * step):
            distance = step - 1
            hit_wall = true
            break
        distance = step
    return {"cells": max(0.0, distance + rng.randfn(0.0, 0.18)), "hit_wall": hit_wall}

func integrate_beam(sensor_heading: int, reading: Dictionary) -> void:
    var free_cells: int = clampi(roundi(reading.cells), 0, 8)
    for step in range(free_cells + 1):
        update_belief(robot + DIRECTIONS[sensor_heading] * step, -1)
    if reading.hit_wall and free_cells < 8:
        update_belief(robot + DIRECTIONS[sensor_heading] * (free_cells + 1), 3)

func update_belief(point: Vector2i, amount: int) -> void:
    if in_bounds(point):
        belief[point.y][point.x] = clampi(belief[point.y][point.x] + amount, -8, 8)

func _draw() -> void:
    draw_string(ThemeDB.fallback_font, Vector2(28, 36), "Artificial Vision Maze Robot", HORIZONTAL_ALIGNMENT_LEFT, -1, 26, Color.WHITE)
    draw_string(ThemeDB.fallback_font, Vector2(28, 62), "Space: pause/resume   R: reset   Frame: %d" % frame, HORIZONTAL_ALIGNMENT_LEFT, -1, 16, Color("b8c4d1"))
    draw_string(ThemeDB.fallback_font, LEFT_ORIGIN + Vector2(0, -14), "Ground truth (debug view)", HORIZONTAL_ALIGNMENT_LEFT, -1, 16, Color("b8c4d1"))
    draw_string(ThemeDB.fallback_font, MAP_ORIGIN + Vector2(0, -14), "Discovered occupancy map", HORIZONTAL_ALIGNMENT_LEFT, -1, 16, Color("b8c4d1"))
    draw_maze(LEFT_ORIGIN)
    draw_belief(MAP_ORIGIN)
    draw_sensor_rays()
    var status := "Sensors — left: %.2f   front: %.2f   right: %.2f" % [readings[0].cells, readings[1].cells, readings[2].cells]
    draw_string(ThemeDB.fallback_font, Vector2(28, 500), status, HORIZONTAL_ALIGNMENT_LEFT, -1, 19, Color("e6edf3"))
    var map_legend := "Occupancy map: dark = unknown, light = free, red = wall"
    draw_string(ThemeDB.fallback_font, Vector2(28, 532), map_legend, HORIZONTAL_ALIGNMENT_LEFT, -1, 16, Color("b8c4d1"))
    if paused:
        draw_string(ThemeDB.fallback_font, Vector2(28, 575), "Paused — press Space to continue" if robot != goal else "Goal reached — press R for a fresh maze", HORIZONTAL_ALIGNMENT_LEFT, -1, 18, Color("ffd166"))

func draw_maze(origin: Vector2) -> void:
    for y in GRID_HEIGHT:
        for x in GRID_WIDTH:
            var point := Vector2i(x, y)
            var color := Color("26313d") if maze_cell(point) == "#" else Color("dbe8f3")
            if point in route and point != robot:
                color = Color("a78bfa")
            if point == goal:
                color = Color("36c995")
            draw_rect(Rect2(origin + Vector2(point) * TILE, Vector2(TILE - 1, TILE - 1)), color)
    draw_robot(origin)

func draw_belief(origin: Vector2) -> void:
    for y in GRID_HEIGHT:
        for x in GRID_WIDTH:
            var point := Vector2i(x, y)
            var value: int = belief[y][x]
            var color := Color("17212b")
            if value <= -2:
                color = Color("dbe8f3")
            elif value >= 2:
                color = Color("d15454")
            draw_rect(Rect2(origin + Vector2(point) * TILE, Vector2(TILE - 1, TILE - 1)), color)
    draw_robot(origin)

func draw_robot(origin: Vector2) -> void:
    var center := origin + Vector2(robot) * TILE + Vector2(TILE / 2.0, TILE / 2.0)
    draw_circle(center, TILE * 0.36, Color("3989ff"))
    draw_line(center, center + Vector2(DIRECTIONS[heading]) * TILE * 0.55, Color.WHITE, 3.0)

func draw_sensor_rays() -> void:
    var start := LEFT_ORIGIN + Vector2(robot) * TILE + Vector2(TILE / 2.0, TILE / 2.0)
    for i in readings.size():
        var sensor_heading: int = [(heading + 3) % 4, heading, (heading + 1) % 4][i]
        var end: Vector2 = start + Vector2(DIRECTIONS[sensor_heading]) * readings[i].cells * TILE
        draw_line(start, end, SENSOR_COLORS[i], 2.0)

func in_bounds(point: Vector2i) -> bool:
    return point.x >= 0 and point.x < GRID_WIDTH and point.y >= 0 and point.y < GRID_HEIGHT

func is_wall(point: Vector2i) -> bool:
    return not in_bounds(point) or maze_cell(point) == "#"

func maze_cell(point: Vector2i) -> String:
    return maze[point.y][point.x]

func set_maze_cell(point: Vector2i, value: String) -> void:
    maze[point.y][point.x] = value

func manhattan(a: Vector2i, b: Vector2i) -> int:
    return absi(a.x - b.x) + absi(a.y - b.y)
