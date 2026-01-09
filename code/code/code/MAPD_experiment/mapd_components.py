import numpy as np
from typing import List, Tuple, Dict, Any, Optional
import math
import heapq

Location = Tuple[int, int]
Path = List[Location]
ReservationTable = Dict[Location, List[int]]

class GridWorldMap:
    def __init__(self, map_file: str):
        self.map_data: List[str] = []
        self.starts: Dict[int, Location] = {}
        self.height = 0
        self.width = 0
        self.load_map(map_file)
        
    def load_map(self, map_file: str):
        with open(map_file, 'r') as f:
            lines = f.readlines()
        
        self.height, self.width = map(int, lines[0].split())
        self.map_data = [list(line.strip()) for line in lines[1:1+self.height]]
        
        for line in lines[1+self.height:]:
            if line.startswith('Start_'):
                parts = line.split(':')
                agent_id = int(parts[0].split('_')[1])
                y, x = map(int, parts[1].strip().split())
                self.starts[agent_id] = (y, x)
                
    def is_traversable(self, loc: Location) -> bool:
        y, x = loc
        if 0 <= y < self.height and 0 <= x < self.width and self.map_data[y][x] != '#':
            return True
        return False
        
    def get_traversable_cells(self) -> List[Location]:
        cells = []
        for r in range(self.height):
            for c in range(self.width):
                if self.map_data[r][c] == '.':
                    cells.append((r, c))
        return cells
        
    def get_start_location(self, agent_id: int) -> Location:
        return self.starts.get(agent_id, (0, 0))

class Task:
    def __init__(self, task_id: int, pickup_loc: Location, delivery_loc: Location, arrival_time: int):
        self.id = task_id
        self.pickup_loc = pickup_loc
        self.delivery_loc = delivery_loc
        self.arrival_time = arrival_time
        self.assigned_time: Optional[int] = None
        self.delivery_time: Optional[int] = None
        
    def __repr__(self) -> str:
        return f"Task(id={self.id}, arr={self.arrival_time}, P={self.pickup_loc}, D={self.delivery_loc})"

class Agent:
    def __init__(self, agent_id: int, start_loc: Location):
        self.id = agent_id
        self.current_loc = start_loc
        self.current_task: Optional[Task] = None
        self.current_path: Path = [start_loc]
        self.path_step = 0
        self.has_package = False
        
    def is_busy(self) -> bool:
        return self.current_task is not None
    
    def is_moving(self) -> bool:
        return self.path_step < len(self.current_path) - 1
        
    def move_one_step(self):
        if self.is_moving():
            self.path_step += 1
            self.current_loc = self.current_path[self.path_step]
        
    def assign_task(self, task: Task):
        self.current_task = task
        self.has_package = False
        self.path_step = 0
        
    def complete_task(self):
        if self.current_task:
            self.current_task.delivery_time = self.path_step
        self.current_task = None
        self.has_package = False
        self.current_path = [self.current_loc]
        self.path_step = 0
        
    def is_at_pickup(self) -> bool:
        return (self.current_task and 
                self.current_loc == self.current_task.pickup_loc and 
                not self.has_package)

    def is_at_delivery(self) -> bool:
        return (self.current_task and 
                self.current_loc == self.current_task.delivery_loc and 
                self.has_package)
                
    def set_path(self, path: Path):
        self.current_path = path
        self.path_step = 0

        if self.current_task and self.current_loc == self.current_task.pickup_loc:
            self.has_package = True

class ClosestAvailableAgentScheduler:
    def __init__(self):
        pass

    def manhattan_distance(self, loc1: Location, loc2: Location) -> int:
        return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])

    def assign_tasks(self, available_agents: List[Agent], new_tasks: List[Task]) -> Dict[int, Task]:
        assignments: Dict[int, Task] = {}

        sorted_tasks = sorted(new_tasks, key=lambda t: t.arrival_time)
        
        for task in sorted_tasks:
            best_agent: Optional[Agent] = None
            min_dist = float('inf')

            for agent in available_agents:
                dist = self.manhattan_distance(agent.current_loc, task.pickup_loc)
                
                if dist < min_dist:
                    min_dist = dist
                    best_agent = agent
                    
            if best_agent:
                assignments[best_agent.id] = task
                available_agents.remove(best_agent)
                
        return assignments