import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    table = {}
    for c in constraints:
        t = c['timestep']

        if c.get('positive', False):
            if c['agent'] != agent:
                continue
        elif c['agent'] != agent: 
            continue

        if t not in table:
            table[t] = []

        table[t].append(c)
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    if next_time not in constraint_table:
        return False

    must_be_locs = []
    must_take_edges = []

    for c in constraint_table[next_time]:
        is_positive = c.get('positive', False)

        if not is_positive and len(c['loc']) == 1:
            if c['loc'][0] == next_loc:
                return True

        elif not is_positive and len(c['loc']) == 2:
            if c['loc'][0] == curr_loc and c['loc'][1] == next_loc:
                return True

        elif is_positive and len(c['loc']) == 1:
            must_be_locs.append(c['loc'][0])

        elif is_positive and len(c['loc']) == 2:
            must_take_edges.append(tuple(c['loc']))

    if must_be_locs and next_loc not in must_be_locs:
        return True

    if must_take_edges and (curr_loc, next_loc) not in must_take_edges:
        return True

    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    
    # Increased time limit for MAPD environments
    MAX_TIMESTEP = 5000 
    
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints,agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        
        # If the path is getting extremely long, prune it.
        # But in MAPD, we might be starting at time_step=1000, 
        # so we check g_val (length of THIS search), not time_step.
        if curr['g_val'] >= MAX_TIMESTEP:
            continue
            
        if curr['loc'] == goal_loc:
            return get_path(curr)

        for dir in range(5):
            if dir < 4:
                child_loc = move(curr['loc'], dir)
            else:
                child_loc = curr['loc']

            if not (0 <= child_loc[0] < len(my_map) and 0 <= child_loc[1] < len(my_map[0])):
                continue

            if my_map[child_loc[0]][child_loc[1]]:
                continue

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step': curr['time_step'] + 1}
            
            if is_constrained(curr['loc'],child_loc, child['time_step'], constraint_table):
                continue
            
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)
                

    return None