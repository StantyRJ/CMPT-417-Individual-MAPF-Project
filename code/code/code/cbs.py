import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    max_t = max(len(path1), len(path2))

    for t in range(max_t):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)

        if loc1 == loc2:
            return {'loc': [loc1], 'timestep': t}

        if t < max_t - 1:
            next1 = get_location(path1, t + 1)
            next2 = get_location(path2, t + 1)
            if loc1 == next2 and loc2 == next1:
                return {'loc': [loc1, next1], 'timestep': t + 1}

    return None


def detect_collisions(paths):
    collisions = []
    max_len = max(len(p) for p in paths) if paths else 0
    
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision:
                collisions.append({
                    'a1': i, 
                    'a2': j, 
                    'loc': collision['loc'], 
                    'timestep': collision['timestep']
                })
    return collisions


def standard_splitting(collision):
    constraints = []
    a1 = collision['a1']
    a2 = collision['a2']
    loc = collision['loc']
    timestep = collision['timestep']

    if len(loc) == 1:
        constraints.append({'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        constraints.append({'agent': a2, 'loc': loc, 'timestep': timestep, 'positive': False})
    elif len(loc) == 2:
        constraints.append({'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        constraints.append({'agent': a2, 'loc': [loc[1], loc[0]], 'timestep': timestep, 'positive': False})

    return constraints


class CBSSolver(object):
    def __init__(self, my_map, starts, goals, tasks):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.tasks = tasks
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.open_list = []

    def push_node(self, node):
        heapq.heappush(self.open_list, (
            node['cost'], 
            len(node['collisions']), 
            self.num_of_generated, 
            node
        ))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def plan_path_segment(self, agent_id, start_loc, goal_loc, constraints):
        """Plan a path segment for an agent"""
        h_values = compute_heuristics(self.my_map, goal_loc)
        path = a_star(self.my_map, start_loc, goal_loc, h_values, agent_id, constraints)
        return path

    def replan_agent_from_point(self, agent_id, paths, constraints, start_time):
        """Replan agent path from a specific timepoint forward"""
        current_path = paths[agent_id]
        
        # In MAPD with tight corridors, replanning from the collision time often fails
        # because the agent is already trapped in the corridor.
        # We defaults to replanning from the very beginning (start_time=0) to allow 
        # the agent to wait *before* entering the bottleneck.
        
        if start_time >= len(current_path):
            start_loc = current_path[-1]
        else:
            start_loc = current_path[start_time]
        
        # Goal is the final location
        goal_loc = current_path[-1]
        
        # Plan from start_loc to goal_loc
        h_values = compute_heuristics(self.my_map, goal_loc)
        new_segment = a_star(self.my_map, start_loc, goal_loc, h_values, agent_id, constraints)
        
        if new_segment is None:
            return None
        
        # Combine: keep path up to start_time, then append new segment
        if start_time == 0:
            return new_segment
        else:
            return current_path[:start_time] + new_segment[1:]  # Skip duplicate first location

    def find_solution(self, disjoint=False):
        self.start_time = timer.time()
        
        # Initialize with agents at starting positions
        agent_paths = [[start] for start in self.starts]
        current_agent_locs = list(self.starts)
        unassigned_tasks = list(self.tasks)

        root = {
            'cost': 0,
            'constraints': [],
            'paths': agent_paths,
            'collisions': [],
            'current_agent_locs': current_agent_locs,
            'agent_availability_time': [0] * self.num_of_agents,
            'unassigned_tasks': unassigned_tasks
        }
        
        self.push_node(root)
        
        MAX_CBS_NODES = 500000 # Increased for MAPD complexity
        best_solution = None

        while len(self.open_list) > 0:
            if self.num_of_generated > MAX_CBS_NODES:
                print(f"CBS node limit reached. Generated: {self.num_of_generated}")
                if best_solution:
                    print("Returning best partial solution found")
                    return best_solution['paths']
                return None
                
            P = self.pop_node()

            # Solution found
            if not P['collisions'] and not P['unassigned_tasks']:
                self.print_results(P)
                return P['paths']
            
            # Track best solution with all tasks assigned
            if not P['unassigned_tasks']:
                if best_solution is None or P['cost'] < best_solution['cost']:
                    best_solution = P
            
            # Handle collisions first (higher priority)
            if P['collisions']:
                collision = P['collisions'][0]
                constraints = standard_splitting(collision)

                for constraint in constraints:
                    Q = {
                        'constraints': P['constraints'] + [constraint],
                        'paths': [list(p) for p in P['paths']],
                        'current_agent_locs': list(P['current_agent_locs']),
                        'agent_availability_time': list(P['agent_availability_time']),
                        'unassigned_tasks': list(P['unassigned_tasks'])
                    }

                    agent_id = constraint['agent']
                    collision_time = constraint['timestep']
                    
                    # FIX: Always replan from 0 to avoid getting trapped in corridors
                    # If we replan from collision_time - 1, the agent might already be 
                    # inside a 1-way tunnel and have no valid moves.
                    replan_from = 0 
                    
                    new_path = self.replan_agent_from_point(
                        agent_id, P['paths'], Q['constraints'], replan_from
                    )

                    if new_path is not None:
                        Q['paths'][agent_id] = new_path
                        Q['agent_availability_time'][agent_id] = len(new_path) - 1
                        Q['current_agent_locs'][agent_id] = new_path[-1]
                        
                        Q['collisions'] = detect_collisions(Q['paths'])
                        Q['cost'] = get_sum_of_cost(Q['paths'])
                        self.push_node(Q)
                continue

            # Assign tasks when no collisions
            if not P['collisions'] and P['unassigned_tasks']:
                # Find best single assignment
                best_assignment = None
                min_cost = float('inf')

                for task in P['unassigned_tasks']:
                    required_start_t = task.get('arrival_time', 0)
                    pickup_loc = task['pickup_loc']
                    delivery_loc = task['delivery_loc']
                    
                    for i in range(self.num_of_agents):
                        agent_available = P['agent_availability_time'][i]
                        actual_start = max(agent_available, required_start_t)
                        
                        # Plan to pickup
                        path_to_pickup = self.plan_path_segment(
                            i, P['current_agent_locs'][i], pickup_loc, P['constraints']
                        )
                        
                        if not path_to_pickup:
                            continue
                            
                        # Plan to delivery
                        path_to_delivery = self.plan_path_segment(
                            i, pickup_loc, delivery_loc, P['constraints']
                        )
                        
                        if not path_to_delivery:
                            continue
                        
                        # Calculate cost
                        travel_to_pickup = len(path_to_pickup) - 1
                        travel_to_delivery = len(path_to_delivery) - 1
                        completion_time = actual_start + travel_to_pickup + travel_to_delivery
                        
                        # Heuristic: prefer shorter total time
                        cost = completion_time
                        
                        if cost < min_cost:
                            min_cost = cost
                            best_assignment = {
                                'agent_id': i,
                                'task': task,
                                'path_to_pickup': path_to_pickup,
                                'path_to_delivery': path_to_delivery,
                                'actual_start': actual_start
                            }

                if best_assignment is None:
                    continue  # No feasible assignment
                
                # Create new node with this assignment
                Q = {
                    'constraints': list(P['constraints']),
                    'paths': [list(p) for p in P['paths']],
                    'current_agent_locs': list(P['current_agent_locs']),
                    'agent_availability_time': list(P['agent_availability_time']),
                    'unassigned_tasks': [t for t in P['unassigned_tasks'] 
                                        if t != best_assignment['task']]
                }
                
                agent_id = best_assignment['agent_id']
                actual_start = best_assignment['actual_start']
                
                # Build full path
                current_path = Q['paths'][agent_id]
                
                # Add waiting if needed
                wait_needed = max(0, actual_start + 1 - len(current_path))
                if wait_needed > 0:
                    wait_loc = current_path[-1]
                    current_path.extend([wait_loc] * wait_needed)
                
                # Add movement
                full_moves = (best_assignment['path_to_pickup'][1:] + 
                             best_assignment['path_to_delivery'][1:])
                current_path.extend(full_moves)
                
                Q['paths'][agent_id] = current_path
                Q['agent_availability_time'][agent_id] = len(current_path) - 1
                Q['current_agent_locs'][agent_id] = current_path[-1]
                
                # Check for collisions
                Q['collisions'] = detect_collisions(Q['paths'])
                Q['cost'] = get_sum_of_cost(Q['paths'])
                self.push_node(Q)
                
        if best_solution:
            print("\nReturning best solution found (may have collisions)")
            self.print_results(best_solution)
            return best_solution['paths']
            
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        if node['collisions']:
            print(f"WARNING: Solution has {len(node['collisions'])} collisions")