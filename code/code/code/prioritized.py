import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_location
import heapq


class PrioritizedPlanningSolver(object):
    def __init__(self, my_map, starts, goals, tasks):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.tasks = tasks
        self.num_of_agents = len(goals)

        self.CPU_time = 0

    def find_solution(self):
        start_time = timer.time()
        
        agent_paths = [[start] for start in self.starts] 
        agent_availability_time = [0] * self.num_of_agents 
        
        constraints = []
        unassigned_tasks = list(self.tasks)
        
        current_agent_locs = list(self.starts)

        while unassigned_tasks:
            
            best_assignment = None
            min_total_time = float('inf') 
            
            for task in unassigned_tasks:
                required_start_t = task['arrival_time']
                
                for i in range(self.num_of_agents):
                    
                    agent_current_loc = current_agent_locs[i]
                    agent_available_at = agent_availability_time[i] 
                    
                    actual_path_start_t = max(agent_available_at, required_start_t)
                    
                    h_pickup = compute_heuristics(self.my_map, task['pickup_loc'])

                    path_to_pickup = a_star(
                        self.my_map, agent_current_loc, task['pickup_loc'], 
                        h_pickup, i, constraints
                    )
                    
                    if path_to_pickup:
                        travel_time_to_pickup = len(path_to_pickup) - 1
                        
                        time_at_pickup = actual_path_start_t + travel_time_to_pickup
                        
                        h_delivery = compute_heuristics(self.my_map, task['delivery_loc'])

                        path_to_delivery = a_star(
                            self.my_map, task['pickup_loc'], task['delivery_loc'], 
                            h_delivery, i, constraints
                        )
                        
                        if path_to_delivery:
                            T_completion = time_at_pickup + (len(path_to_delivery) - 1)
                        
                            if T_completion < min_total_time:
                                min_total_time = T_completion
                                best_assignment = {
                                    'agent_id': i, 
                                    'task': task, 
                                    'actual_path_start_t': actual_path_start_t,
                                    'path_to_pickup': path_to_pickup,
                                    'path_to_delivery': path_to_delivery
                                }

            if best_assignment is None:
                break 

            agent_id = best_assignment['agent_id']
            task = best_assignment['task']
            actual_path_start_t = best_assignment['actual_path_start_t']
            
            path_to_pickup_moves = best_assignment['path_to_pickup'][1:] 
            path_to_delivery_moves = best_assignment['path_to_delivery'][1:]
            
            full_path_moves = path_to_pickup_moves + path_to_delivery_moves
            
            current_path_length = len(agent_paths[agent_id])
            required_start_t = actual_path_start_t
            
            if current_path_length < required_start_t + 1:
                wait_loc = agent_paths[agent_id][-1]
                for _ in range(required_start_t + 1 - current_path_length):
                    agent_paths[agent_id].append(wait_loc)
            
            agent_paths[agent_id].extend(full_path_moves)
            
            agent_availability_time[agent_id] = len(agent_paths[agent_id]) - 1
            current_agent_locs[agent_id] = agent_paths[agent_id][-1]
            
            start_t = required_start_t + 1

            for t, loc in enumerate(full_path_moves, start=start_t):
                v_constraint = {'agent': agent_id, 'loc': [loc], 'timestep': t, 'positive': False}
                constraints.append(v_constraint)

                if t > start_t:
                    prev_loc = get_location(agent_paths[agent_id], t - 1)
                    curr_loc = loc
                    e_constraint = {'agent': agent_id, 'loc': [curr_loc, prev_loc], 'timestep': t, 'positive': False}
                    constraints.append(e_constraint)

            unassigned_tasks.remove(task)
        
        result = agent_paths
        
        max_len = 0
        if result:
            for path in result:
                max_len = max(max_len, len(path))
        
        for i in range(len(result)):
            path = result[i]
            if len(path) < max_len:
                last_loc = path[-1]
                for _ in range(max_len - len(path)):
                    path.append(last_loc)
        

        self.CPU_time = timer.time() - start_time

        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        return result