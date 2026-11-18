"""Deterministic grid planning algorithms and a lightweight mobile robot simulation."""
from __future__ import annotations
from dataclasses import dataclass
from heapq import heappush, heappop
from typing import Iterable

Cell = tuple[int, int]

@dataclass
class Grid:
    width: int
    height: int
    blocked: set[Cell]

    def __post_init__(self):
        if self.width < 1 or self.height < 1:
            raise ValueError('Grid dimensions must be positive')
        if any(not self.inside(cell) for cell in self.blocked):
            raise ValueError('Obstacle outside map')

    def inside(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def free(self, cell: Cell) -> bool:
        return self.inside(cell) and cell not in self.blocked

    def neighbors(self, cell: Cell) -> Iterable[Cell]:
        x, y = cell
        for point in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if self.free(point):
                yield point

def plan(grid: Grid, start: Cell, goal: Cell, algorithm: str = 'astar') -> list[Cell]:
    """Return shortest four-connected path, including endpoints, or [] if unreachable."""
    if algorithm not in ('astar', 'dijkstra'):
        raise ValueError('Unknown algorithm')
    if not grid.free(start) or not grid.free(goal):
        return []
    heap = [(0, 0, start)]
    came: dict[Cell, Cell] = {}
    costs = {start: 0}
    while heap:
        _, cost, cell = heappop(heap)
        if cost != costs.get(cell):
            continue
        if cell == goal:
            path = [goal]
            while path[-1] != start:
                path.append(came[path[-1]])
            return list(reversed(path))
        for neighbor in grid.neighbors(cell):
            candidate = cost + 1
            if candidate < costs.get(neighbor, float('inf')):
                costs[neighbor] = candidate
                came[neighbor] = cell
                heuristic = abs(goal[0] - neighbor[0]) + abs(goal[1] - neighbor[1]) if algorithm == 'astar' else 0
                heappush(heap, (candidate + heuristic, candidate, neighbor))
    return []

@dataclass
class Robot:
    grid: Grid
    position: Cell
    target: Cell
    algorithm: str = 'astar'

    def __post_init__(self):
        self.path = plan(self.grid, self.position, self.target, self.algorithm)
        self.travelled = [self.position]

    def step(self) -> bool:
        """Move one cell, dynamically replanning after obstacle changes."""
        self.path = plan(self.grid, self.position, self.target, self.algorithm)
        if len(self.path) < 2:
            return False
        self.position = self.path[1]
        self.travelled.append(self.position)
        return True

    def add_obstacle(self, cell: Cell):
        if cell != self.position and cell != self.target and self.grid.inside(cell):
            self.grid.blocked.add(cell)
            self.path = plan(self.grid, self.position, self.target, self.algorithm)

def parse_ascii(text: str) -> tuple[Grid, Cell, Cell]:
    lines = [line for line in text.splitlines() if line.strip()]
    if not lines or len(set(map(len, lines))) != 1:
        raise ValueError('Map must be a nonempty rectangle')
    starts = [(x,y) for y,row in enumerate(lines) for x,ch in enumerate(row) if ch == 'S']
    goals = [(x,y) for y,row in enumerate(lines) for x,ch in enumerate(row) if ch == 'G']
    if len(starts)!=1 or len(goals)!=1 or any(ch not in '#.SG' for row in lines for ch in row):
        raise ValueError('Map must contain one S, one G, and only #.SG')
    return Grid(len(lines[0]),len(lines),{(x,y) for y,row in enumerate(lines) for x,ch in enumerate(row) if ch=='#'}), starts[0],goals[0]

def render(grid: Grid, path: list[Cell], start: Cell, goal: Cell) -> str:
    marks = set(path)
    return '\n'.join(''.join('S' if (x,y)==start else 'G' if (x,y)==goal else '#' if (x,y) in grid.blocked else '*' if (x,y) in marks else '.' for x in range(grid.width)) for y in range(grid.height))
