#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

Colors = ['green', 'blue', 'orange', 'purple', 'cyan', 'magenta', 'yellow', 'brown']


class Animation:
    def __init__(self, my_map, starts, goals, paths, tasks):
        self.my_map = np.flip(np.transpose(my_map), 1)
        
        self.starts = []
        for start in starts:
            self.starts.append((start[1], len(self.my_map[0]) - 1 - start[0]))
        self.goals = []
        for goal in goals:
            self.goals.append((goal[1], len(self.my_map[0]) - 1 - goal[0]))
            
        self.tasks = []
        
        if tasks and isinstance(tasks[0], dict) and ('pickup' in tasks[0] or 'pickup_loc' in tasks[0]):
            self.tasks = tasks
        elif goals and goals != self.starts:
            for i, start_loc in enumerate(starts):
                delivery_loc = goals[i] if i < len(goals) else start_loc
                self.tasks.append({
                    'id': i,
                    'arrival_time': 0,
                    'pickup_loc': start_loc, 
                    'delivery_loc': delivery_loc
                })
        else:
             for i, start_loc in enumerate(starts):
                 delivery_loc = paths[i][-1] if paths and len(paths[i]) > 0 else start_loc
                 self.tasks.append({
                    'id': i,
                    'arrival_time': 0,
                    'pickup_loc': start_loc, 
                    'delivery_loc': delivery_loc
                 })


        self.task_patches = {}
        self.task_states = {}
        self.task_labels = {}
        
        for task in self.tasks:
            pr = task.get('pickup_loc', task.get('pickup'))[0]
            pc = task.get('pickup_loc', task.get('pickup'))[1]
            dr = task.get('delivery_loc', task.get('delivery'))[0]
            dc = task.get('delivery_loc', task.get('delivery'))[1]
            
            flipped_pickup = (pc, len(self.my_map[0]) - 1 - pr)
            flipped_delivery = (dc, len(self.my_map[0]) - 1 - dr)
            
            task['flipped_pickup'] = flipped_pickup
            task['flipped_delivery'] = flipped_delivery
            
            self.task_states[task['id']] = {'pickup_t': -1, 'delivery_t': -1, 'assigned_agent_id': -1}


        self.paths = []
        if paths:
            for path in paths:
                self.paths.append([])
                for loc in path:
                    self.paths[-1].append((loc[1], len(self.my_map[0]) - 1 - loc[0]))
        
        self.T = 0
        if self.paths:
            for path in self.paths:
                self.T = max(self.T, len(path) - 1)


        aspect = len(self.my_map) / len(self.my_map[0])

        self.fig = plt.figure(frameon=False, figsize=(8 * aspect, 8))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()

        padding = 1.0
        x_min = -0.5 - padding
        y_min = -0.5 - padding
        x_max = len(self.my_map) - 0.5 + padding
        y_max = len(self.my_map[0]) - 0.5 + padding
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='black', edgecolor='gray'))

        self.patches.append(Rectangle((-0.5, -0.5), len(self.my_map), len(self.my_map[0]), 
                                       facecolor='white', edgecolor='gray'))

        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j]:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))


        for i, task in enumerate(self.tasks):
            p_patch = Circle(task['flipped_pickup'], 0.35, facecolor='black', edgecolor='gray', alpha=0.8)
            self.patches.append(p_patch)
            self.task_patches[(task['id'], 'pickup')] = p_patch
            p_patch.set_visible(False)
            
            p_label = self.ax.text(task['flipped_pickup'][0], task['flipped_pickup'][1], str(task['id']),
                                    fontsize=8, ha='center', va='center', weight='bold', zorder=4, color='white')
            self.task_labels[f'P{task["id"]}'] = p_label
            self.artists.append(p_label)
            p_label.set_visible(False)

            d_patch = Rectangle((task['flipped_delivery'][0] - 0.4, task['flipped_delivery'][1] - 0.4), 0.8, 0.8, 
                                 facecolor=Colors[i % len(Colors)], edgecolor='black', alpha=0.5)
            self.patches.append(d_patch)
            self.task_patches[(task['id'], 'delivery')] = d_patch
            d_patch.set_visible(False)
            
            d_label = self.ax.text(task['flipped_delivery'][0], task['flipped_delivery'][1], str(task['id']),
                                    fontsize=8, ha='center', va='center', weight='bold', zorder=4, color='black')
            self.task_labels[f'D{task["id"]}'] = d_label
            self.artists.append(d_label)
            d_label.set_visible(False)


        for i in range(len(self.paths)):
            name = str(i)
            self.agents[i] = Circle((self.starts[i][0], self.starts[i][1]), 0.3, facecolor=Colors[i % len(Colors)],
                                     edgecolor='black', zorder=5)
            self.agents[i].original_face_color = Colors[i % len(Colors)]
            self.patches.append(self.agents[i])
            
            self.agent_names[i] = self.ax.text(self.starts[i][0], self.starts[i][1], name,
                                                fontsize=10, ha='center', va='center',
                                                weight='bold', zorder=6, color='black')
            self.artists.append(self.agent_names[i])
        
        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 1) * 10,
                                                 interval=100,
                                                 blit=True)

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    @staticmethod
    def show():
        plt.show()

    def reset_task_state(self):
        for task in self.tasks:
            task_id = task['id']
            self.task_states[task_id] = {'pickup_t': -1, 'delivery_t': -1, 'assigned_agent_id': -1}
            
            p_patch = self.task_patches.get((task_id, 'pickup'))
            d_patch = self.task_patches.get((task_id, 'delivery'))
            p_label = self.task_labels.get(f'P{task_id}')
            d_label = self.task_labels.get(f'D{task_id}')
            
            if p_patch:
                p_patch.set_visible(False) 
            if d_patch:
                d_patch.set_visible(False)
            if p_label:
                p_label.set_visible(False)
            if d_label:
                d_label.set_visible(False)

    def init_func(self):
        self.reset_task_state() 
        
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        time = t / 10.0
        current_t = int(time)
        
        agent_carrying = {} 
        
        for k in range(len(self.paths)):
            pos = self.get_state(time, self.paths[k])
            self.agents[k].center = (pos[0], pos[1])
            self.agent_names[k].set_position((pos[0], pos[1]))

        for task in self.tasks:
            task_id = task['id']
            p_patch = self.task_patches.get((task_id, 'pickup'))
            d_patch = self.task_patches.get((task_id, 'delivery'))
            p_label = self.task_labels.get(f'P{task_id}')
            d_label = self.task_labels.get(f'D{task_id}')

            if current_t >= task.get('arrival_time', 0):
                if p_patch and self.task_states[task_id]['pickup_t'] == -1:
                    p_patch.set_visible(True)
                    if p_label: p_label.set_visible(True)
                if d_patch and self.task_states[task_id]['delivery_t'] == -1:
                    d_patch.set_visible(True)
                    if d_label: d_label.set_visible(True)

            if p_patch and self.task_states[task_id]['pickup_t'] == -1 and current_t >= task.get('arrival_time', 0):
                pickup_loc_viz = task['flipped_pickup']
                for agent_id, path in enumerate(self.paths):
                    if current_t < len(path) and path[current_t] == pickup_loc_viz:
                        self.task_states[task_id]['pickup_t'] = current_t
                        self.task_states[task_id]['assigned_agent_id'] = agent_id
                        break

            assigned_id = self.task_states[task_id]['assigned_agent_id']
            
            if d_patch and self.task_states[task_id]['pickup_t'] != -1 and self.task_states[task_id]['delivery_t'] == -1:
                delivery_loc_viz = task['flipped_delivery']
                
                if assigned_id != -1:
                    path = self.paths[assigned_id]
                    if current_t < len(path) and path[current_t] == delivery_loc_viz:
                        self.task_states[task_id]['delivery_t'] = current_t

            if self.task_states[task_id]['pickup_t'] != -1 and self.task_states[task_id]['delivery_t'] == -1:
                agent_carrying[assigned_id] = task_id
                if p_patch: p_patch.set_visible(False)
                if p_label: p_label.set_visible(False)

            if self.task_states[task_id]['delivery_t'] != -1:
                delivery_time = self.task_states[task_id]['delivery_t']
                if current_t > delivery_time:
                    if d_patch: d_patch.set_visible(False)
                    if d_label: d_label.set_visible(False)
        
        for agent_id, agent in self.agents.items():
            if agent_id in agent_carrying:
                agent.set_facecolor('brown')
                self.agent_names[agent_id].set_color('white')
            else:
                agent.set_facecolor(agent.original_face_color)
                self.agent_names[agent_id].set_color('black')

        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    self.agent_names[i].set_color('white')
                    self.agent_names[j].set_color('white')

        return self.patches + self.artists

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos