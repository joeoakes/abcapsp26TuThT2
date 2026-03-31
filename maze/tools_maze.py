"""
tools_maze.py

Maze utilities:
- legal_moves(width, height, cells, x, y)
- validate_plan(plan, width, height, cells, start_x, start_y)
- astar(width, height, cells, start, goal)

Runtime Redis keys managed here:
- team2ffmaze:{session_id}:plan
- team2ffmaze:{session_id}:plan_index
"""

from __future__ import annotations

import heapq
import json
from typing import Dict, List, Sequence, Set, Tuple

import redis


WALL_N = 1
WALL_E = 2
WALL_S = 4
WALL_W = 8

MOVE_DELTAS: Dict[str, Tuple[int, int, int]] = {
    "UP": (0, -1, WALL_N),
    "RIGHT": (1, 0, WALL_E),
    "DOWN": (0, 1, WALL_S),
    "LEFT": (-1, 0, WALL_W),
}

OPPOSITE_WALL = {
    "UP": WALL_S,
    "RIGHT": WALL_W,
    "DOWN": WALL_N,
    "LEFT": WALL_E,
}


def session_key(session_id: str, field: str) -> str:
    return f"team2ffmaze:{session_id}:{field}"


def get_redis(host: str = "127.0.0.1", port: int = 6379, db: int = 0) -> redis.Redis:
    return redis.Redis(host=host, port=port, db=db, decode_responses=True)


def _cell_index(width: int, x: int, y: int) -> int:
    return y * width + x


def _in_bounds(width: int, height: int, x: int, y: int) -> bool:
    return 0 <= x < width and 0 <= y < height


def _normalize_cells(width: int, height: int, cells: Sequence[int]) -> List[int]:
    cells_list = list(cells)
    expected = width * height

    if len(cells_list) != expected:
        raise ValueError(f"cells length mismatch: expected {expected}, got {len(cells_list)}")

    for i, value in enumerate(cells_list):
        if not isinstance(value, int):
            raise TypeError(f"cells[{i}] must be int, got {type(value).__name__}")

    return cells_list


def legal_moves(width: int, height: int, cells: Sequence[int], x: int, y: int) -> List[str]:
    cells_list = _normalize_cells(width, height, cells)

    if not _in_bounds(width, height, x, y):
        raise ValueError(f"position out of bounds: ({x}, {y})")

    current = cells_list[_cell_index(width, x, y)]
    allowed: List[str] = []

    for move, (dx, dy, wall_bit) in MOVE_DELTAS.items():
        nx = x + dx
        ny = y + dy

        if not _in_bounds(width, height, nx, ny):
            continue

        if current & wall_bit:
            continue

        neighbor = cells_list[_cell_index(width, nx, ny)]
        if neighbor & OPPOSITE_WALL[move]:
            continue

        allowed.append(move)

    return allowed


def validate_plan(
    plan: Sequence[str],
    width: int,
    height: int,
    cells: Sequence[int],
    start_x: int,
    start_y: int,
) -> bool:
    if not isinstance(plan, (list, tuple)):
        raise TypeError("plan must be a list or tuple of move strings")

    _normalize_cells(width, height, cells)

    x = start_x
    y = start_y

    if not _in_bounds(width, height, x, y):
        raise ValueError(f"start position out of bounds: ({x}, {y})")

    for i, move in enumerate(plan):
        if not isinstance(move, str):
            raise ValueError(f"plan[{i}] is not a string: {move!r}")

        move = move.upper()
        if move not in MOVE_DELTAS:
            raise ValueError(
                f"plan[{i}] invalid move {move!r}; must be one of {list(MOVE_DELTAS.keys())}"
            )

        allowed = legal_moves(width, height, cells, x, y)
        if move not in allowed:
            raise ValueError(
                f"plan[{i}] move {move!r} is illegal from ({x}, {y}); allowed={allowed}"
            )

        dx, dy, _ = MOVE_DELTAS[move]
        x += dx
        y += dy

    return True


def astar(
    width: int,
    height: int,
    cells: Sequence[int],
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> List[str]:
    cells_list = _normalize_cells(width, height, cells)
    sx, sy = start
    gx, gy = goal

    if not _in_bounds(width, height, sx, sy):
        raise ValueError(f"start out of bounds: {start}")
    if not _in_bounds(width, height, gx, gy):
        raise ValueError(f"goal out of bounds: {goal}")

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_heap: List[Tuple[int, int, Tuple[int, int]]] = []
    heapq.heappush(open_heap, (heuristic(start, goal), 0, start))

    gscore: Dict[Tuple[int, int], int] = {start: 0}
    came_from: Dict[Tuple[int, int], Tuple[Tuple[int, int], str]] = {}
    closed: Set[Tuple[int, int]] = set()

    while open_heap:
        _, current_g, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            path: List[str] = []
            node = current
            while node != start:
                prev, move = came_from[node]
                path.append(move)
                node = prev
            path.reverse()
            return path

        cx, cy = current
        for move in legal_moves(width, height, cells_list, cx, cy):
            dx, dy, _ = MOVE_DELTAS[move]
            neighbor = (cx + dx, cy + dy)

            tentative_g = gscore[current] + 1
            if tentative_g < gscore.get(neighbor, float("inf")):
                gscore[neighbor] = tentative_g
                came_from[neighbor] = (current, move)
                fscore = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (fscore, tentative_g, neighbor))

    return []


def load_latest_session_id(r: redis.Redis) -> str:
    session_id = r.get("team2ffmaze:latest_session_id")
    if not session_id:
        raise RuntimeError("No latest session id found in Redis")
    return session_id


def load_static_maze(r: redis.Redis, session_id: str) -> Dict[str, object]:
    fields = ["width", "height", "cells", "start_x", "start_y", "goal_x", "goal_y"]
    raw = {field: r.get(session_key(session_id, field)) for field in fields}

    missing = [field for field, value in raw.items() if value is None]
    if missing:
        raise KeyError(f"missing static keys for session {session_id}: {missing}")

    return {
        "width": int(raw["width"]),
        "height": int(raw["height"]),
        "cells": json.loads(raw["cells"]),
        "start_x": int(raw["start_x"]),
        "start_y": int(raw["start_y"]),
        "goal_x": int(raw["goal_x"]),
        "goal_y": int(raw["goal_y"]),
    }


def store_plan(r: redis.Redis, session_id: str, plan: Sequence[str]) -> None:
    normalized_plan = [str(move).upper() for move in plan]
    r.set(session_key(session_id, "plan"), json.dumps(normalized_plan))
    r.set(session_key(session_id, "plan_index"), 0)


def build_and_store_astar_plan(r: redis.Redis, session_id: str) -> List[str]:
    maze = load_static_maze(r, session_id)

    plan = astar(
        maze["width"],
        maze["height"],
        maze["cells"],
        (maze["start_x"], maze["start_y"]),
        (maze["goal_x"], maze["goal_y"]),
    )

    validate_plan(
        plan,
        maze["width"],
        maze["height"],
        maze["cells"],
        maze["start_x"],
        maze["start_y"],
    )

    store_plan(r, session_id, plan)
    return plan


if __name__ == "__main__":
    r = get_redis()
    session_id = load_latest_session_id(r)
    plan = build_and_store_astar_plan(r, session_id)

    print("session_id:", session_id)
    print("plan:", plan)
