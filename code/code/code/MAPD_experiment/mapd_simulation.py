import time
import random
import numpy as np
import os
import json
from typing import List, Tuple, Dict, Any

from mapd_components import GridWorldMap, Task, Agent, ClosestAvailableAgentScheduler
from mapd_solvers import CBS_MAPD, SpaceTimeAStar

MAP_FILE = "instance_basic_01.txt"
MAX_SIMULATION_TIME = 800
REPETITIONS = 3

EXP1_NUM_TASKS = 50
EXP1_ARRIVAL_RATE = 0.2
EXP2_NUM_AGENTS = 3
EXP2_NUM_TASKS = 20
EXP3_NUM_AGENTS = 3
EXP3_NUM_TASKS = 20

def generate_dynamic_tasks(num_tasks, arrival_rate, grid_map):
    traversable_cells = grid_map.get_traversable_cells()

    inter_arrival_times = np.random.exponential(1.0 / arrival_rate, num_tasks)
    arrival_times = np.cumsum(inter_arrival_times)
    
    tasks = []
    for i in range(num_tasks):
        if len(traversable_cells) < 2:
            raise ValueError("Map is too small to generate tasks.")
            
        pickup, delivery = random.sample(traversable_cells, 2)
        
        task = Task(
            task_id=i, 
            pickup_loc=pickup, 
            delivery_loc=delivery, 
            arrival_time=int(round(arrival_times[i]))
        )
        tasks.append(task)
        
    return tasks

def generate_fixed_task_set(num_tasks, map_file):
    random.seed(42) 
    np.random.seed(42)
    grid_map = GridWorldMap(map_file)

    return generate_dynamic_tasks(num_tasks, arrival_rate=0.5, grid_map=grid_map)

class MAPDSimulation:
    def __init__(self, map_file, num_agents, solver_type):
        self.grid_map = GridWorldMap(map_file)
        self.agents = [Agent(i, self.grid_map.get_start_location(i)) for i in range(num_agents)]
        self.solver_type = solver_type
        
        if solver_type == 'CBS-MAPD':
            self.solver = CBS_MAPD(self.grid_map)
        elif solver_type == 'Space-Time A*':
            self.solver = SpaceTimeAStar(self.grid_map)
        else:
            raise ValueError(f"Unknown solver type: {solver_type}")
            
        self.scheduler = ClosestAvailableAgentScheduler()
        
        self.metrics = {
            'runtime_s': 0.0,
            'completed_tasks': 0,
            'total_conflicts': 0,
            'task_latencies': []
        }
        
    def run(self, all_tasks: List[Task], max_time: int) -> Dict[str, Any]:
        sim_time = 0
        pending_tasks = all_tasks.copy()
        
        start_time_real = time.time()
        
        while sim_time <= max_time:
            newly_arrived = [t for t in pending_tasks if t.arrival_time <= sim_time]
            pending_tasks = [t for t in pending_tasks if t.arrival_time > sim_time]
            unassigned_tasks = newly_arrived 
            
            available_agents = [a for a in self.agents if not a.is_busy()]

            if unassigned_tasks and available_agents:
                assignments = self.scheduler.assign_tasks(available_agents, unassigned_tasks)
                assigned_tasks = list(assignments.values())
                pending_tasks.extend([t for t in unassigned_tasks if t not in assigned_tasks])
                
                for agent_id, task in assignments.items():
                    agent = self.agents[agent_id]
                    agent.assign_task(task)
                    task.assigned_time = sim_time
                    path_to_pickup, conflicts_p = self.solver.plan_path(agent, self.agents)
                    self.metrics['total_conflicts'] += conflicts_p

                    agent.set_path(path_to_pickup)

            for agent in self.agents:
                if agent.is_moving():
                    agent.move_one_step()

                    if agent.current_task and agent.is_at_pickup():
                        agent.has_package = True
                        path_to_delivery, conflicts_d = self.solver.plan_path(agent, self.agents)
                        self.metrics['total_conflicts'] += conflicts_d
                        agent.set_path(path_to_delivery)
                        agent.move_one_step()

                    if agent.current_task and agent.is_at_delivery():
                        latency = sim_time - agent.current_task.arrival_time
                        self.metrics['task_latencies'].append(latency)
                        self.metrics['completed_tasks'] += 1
                        agent.current_task.delivery_time = sim_time
                        agent.complete_task()

            if not pending_tasks and all(not a.is_busy() for a in self.agents):
                break
            
            sim_time += 1
        
        end_time_real = time.time()
        makespan = sim_time
        self.metrics['total_runtime'] = end_time_real - start_time_real
        self.metrics['avg_task_completion_time'] = np.mean(self.metrics['task_latencies']) if self.metrics['task_latencies'] else 0
        self.metrics['makespan'] = makespan
        self.metrics['max_time_reached'] = sim_time >= max_time
        
        return self.metrics

def run_experiment_1(map_file, max_time, num_tasks, arrival_rate):
    print("\nRunning Experiment 1")
    agent_counts = [1, 2, 4, 6, 8]
    solver_type = 'CBS-MAPD'
    all_results = {}

    random.seed(100)
    np.random.seed(100)
    fixed_task_set = generate_dynamic_tasks(num_tasks, arrival_rate, GridWorldMap(map_file))

    for N in agent_counts:
        N_runs_metrics = []
        for i in range(REPETITIONS):
            print(f"  Exp 1: N={N}, Run {i+1}/{REPETITIONS}")
            sim = MAPDSimulation(map_file, N, solver_type)
            run_metrics = sim.run(fixed_task_set, max_time)
            N_runs_metrics.append(run_metrics)

        all_results[N] = {
            'mean_runtime': np.mean([m['total_runtime'] for m in N_runs_metrics]),
            'mean_conflicts': np.mean([m['total_conflicts'] for m in N_runs_metrics]),
            'mean_completed': np.mean([m['completed_tasks'] for m in N_runs_metrics]),
            'mean_makespan': np.mean([m['makespan'] for m in N_runs_metrics]),
            'mean_avg_latency': np.mean([m['avg_task_completion_time'] for m in N_runs_metrics]),
        }
        
    return all_results

def run_experiment_2(map_file, num_agents, num_tasks):
    print("\nRunning Experiment 2")
    arrival_rates = [0.1, 0.5, 1.0, 1.5, 2.0]
    solver_type = 'CBS-MAPD'
    all_results = {}
    MAX_TIME_EXP2 = 10000
    
    for t in arrival_rates:
        t_runs_metrics = []
        for i in range(REPETITIONS):
            random.seed(200 + i)
            np.random.seed(200 + i)
            task_set = generate_dynamic_tasks(num_tasks, t, GridWorldMap(map_file))
            
            print(f"  Exp 2: t={t}, Run {i+1}/{REPETITIONS}")
            sim = MAPDSimulation(map_file, num_agents, solver_type)
            run_metrics = sim.run(task_set, MAX_TIME_EXP2)
            t_runs_metrics.append(run_metrics)

        all_results[t] = {
            'mean_runtime': np.mean([m['total_runtime'] for m in t_runs_metrics]),
            'mean_conflicts': np.mean([m['total_conflicts'] for m in t_runs_metrics]),
            'mean_completed': np.mean([m['completed_tasks'] for m in t_runs_metrics]),
            'mean_avg_latency': np.mean([m['avg_task_completion_time'] for m in t_runs_metrics]),
        }
        
    return all_results

def run_experiment_3(map_file, num_agents, num_tasks):
    print("\nRunning Experiment 3")
    solver_types = ['CBS-MAPD', 'Space-Time A*']
    all_results = {}
    MAX_TIME_EXP3 = 1000 
    fixed_task_set_template = generate_fixed_task_set(num_tasks, map_file)

    for solver in solver_types:
        solver_runs_metrics = []
        for i in range(REPETITIONS):
            print(f"  Exp 3: Solver={solver}, Run {i+1}/{REPETITIONS}")

            task_set_for_run = [Task(t.id, t.pickup_loc, t.delivery_loc, t.arrival_time) for t in fixed_task_set_template]
            
            sim = MAPDSimulation(map_file, num_agents, solver)
            run_metrics = sim.run(task_set_for_run, MAX_TIME_EXP3)
            solver_runs_metrics.append(run_metrics)

        all_results[solver] = {
            'mean_runtime': np.mean([m['total_runtime'] for m in solver_runs_metrics]),
            'mean_conflicts': np.mean([m['total_conflicts'] for m in solver_runs_metrics]),
            'mean_makespan': np.mean([m['makespan'] for m in solver_runs_metrics]),
            'mean_avg_latency': np.mean([m['avg_task_completion_time'] for m in solver_runs_metrics]),
        }
        
    return all_results

def save_results(data, filename="mapd_results.json"):
    with open(filename, 'w') as f:
        json.dump(data, f, indent=4)
    print(f"\nResults saved")

if __name__ == '__main__':
    if not os.path.exists(MAP_FILE):
        print(f"Map not found")
    else:
        all_exp_results: Dict[str, Any] = {}

        all_exp_results['Experiment 1'] = run_experiment_1(
            map_file=MAP_FILE, 
            max_time=MAX_SIMULATION_TIME,
            num_tasks=EXP1_NUM_TASKS,
            arrival_rate=EXP1_ARRIVAL_RATE
        )

        all_exp_results['Experiment 2'] = run_experiment_2(
            map_file=MAP_FILE,
            num_agents=EXP2_NUM_AGENTS,
            num_tasks=EXP2_NUM_TASKS
        )

        all_exp_results['Experiment 3'] = run_experiment_3(
            map_file=MAP_FILE,
            num_agents=EXP3_NUM_AGENTS,
            num_tasks=EXP3_NUM_TASKS
        )
        
        save_results(all_exp_results)
        print("\nAll experiments complete.")
        print("\n")