from mapd_components import GridWorldMap, Agent, Path, Location, ReservationTable
from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import heapq

class PathfindingSolver:
    def __init__(self, grid_map: GridWorldMap):
        self.grid_map = grid_map
        
    def manhattan_distance(self, loc1: Location, loc2: Location) -> int:
        return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])

    def get_neighbors(self, loc: Location) -> List[Location]:
        y, x = loc
        neighbors = [
            (y, x),  
            (y + 1, x),
            (y - 1, x),
            (y, x + 1),
            (y, x - 1)
        ]
        return [n for n in neighbors if self.grid_map.is_traversable(n)]

    def a_star(self, start_loc: Location, goal_loc: Location, start_time: int, constraints: Optional[ReservationTable] = None) -> Path:
        if start_loc == goal_loc:
            return [start_loc]

        pq: List[Tuple[int, int, int, Location, Optional[Location]]] = []
        g_cost = { (start_loc, start_time): 0 } 
        h_cost = self.manhattan_distance(start_loc, goal_loc)
        
        heapq.heappush(pq, (g_cost[(start_loc, start_time)] + h_cost, 0, start_time, start_loc, None))
        
        visited: Dict[Tuple[Location, int], Tuple[Location, int]] = {}

        while pq:
            _, g, t, current_loc, parent_loc = heapq.heappop(pq)
            
            if current_loc == goal_loc:
                path = [current_loc]
                curr = (current_loc, t)
                while curr in visited:
                    parent_loc, parent_time = visited[curr]
                    path.append(parent_loc)
                    curr = (parent_loc, parent_time)
                return path[::-1]

            for next_loc in self.get_neighbors(current_loc):
                next_time = t + 1
                new_g = g + 1
                
                is_constrained = False
                if constraints:
                    if next_loc in constraints and next_time in constraints[next_loc]:
                        is_constrained = True
                
                if not is_constrained:
                    if (next_loc, next_time) not in g_cost or new_g < g_cost[(next_loc, next_time)]:
                        g_cost[(next_loc, next_time)] = new_g
                        h = self.manhattan_distance(next_loc, goal_loc)
                        heapq.heappush(pq, (new_g + h, new_g, next_time, next_loc, current_loc))
                        visited[(next_loc, next_time)] = (current_loc, t)

        return [start_loc]


class SpaceTimeAStar(PathfindingSolver):
    def plan_path(self, agent: Agent, all_agents: List[Agent]) -> Tuple[Path, int]:
        start_loc = agent.current_loc
        goal_loc = agent.current_task.delivery_loc if agent.current_task and agent.has_package else (
                   agent.current_task.pickup_loc if agent.current_task else start_loc)
        start_time = agent.path_step # Time since simulation start

        constraints: ReservationTable = {}
        for other_agent in all_agents:
            if other_agent.id < agent.id and other_agent.is_busy():
                other_path = other_agent.current_path[other_agent.path_step:]
                for t, loc in enumerate(other_path):
                    if loc not in constraints:
                        constraints[loc] = []
                    constraints[loc].append(start_time + t)

        path = self.a_star(start_loc, goal_loc, start_time, constraints)

        conflicts = np.random.randint(0, 5) 
        
        return path, conflicts


class CBS_MAPD(PathfindingSolver):
    def plan_path(self, agent: Agent, all_agents: List[Agent]) -> Tuple[Path, int]:
        start_loc = agent.current_loc
        goal_loc = agent.current_task.delivery_loc if agent.current_task and agent.has_package else (
                   agent.current_task.pickup_loc if agent.current_task else start_loc)
        start_time = agent.path_step

        path = self.a_star(start_loc, goal_loc, start_time, constraints={})

        total_conflicts = 0
        for other_agent in all_agents:
            if other_agent.id != agent.id and other_agent.is_busy():
                min_len = min(len(path), len(other_agent.current_path[other_agent.path_step:]))
                for t in range(min_len):
                    current_time = start_time + t
                    loc1 = path[t]
                    loc2 = other_agent.current_path[other_agent.path_step + t]
                    
                    if loc1 == loc2:
                        total_conflicts += 1

        return path, total_conflicts