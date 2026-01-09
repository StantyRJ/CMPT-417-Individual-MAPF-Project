import json
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

RESULTS_FILE = 'mapd_results.json'

def load_results(filename):
    if not os.path.exists(filename):
        print(f"Run mapd_simulation first")
    
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def plot_experiment_1(data):
    df = pd.DataFrame.from_dict(data, orient='index')
    df.index = df.index.astype(int)
    df = df.sort_index()

    metrics = {
        'mean_runtime': 'Total Runtime (s)',
        'mean_conflicts': 'Total Conflicts',
        'mean_completed': 'Completed Tasks',
        'mean_avg_latency': 'Avg. Task Completion Time'
    }

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.flatten()
    fig.suptitle('Experiment 1: Scaling Performance with Number of Agents ($N$)', fontsize=16)
    
    for i, (col, title) in enumerate(metrics.items()):
        ax = axes[i]
        if col in df.columns:
            ax.plot(df.index, df[col], marker='o', linestyle='-', linewidth=2, color='tab:blue')
            ax.set_title(title, fontsize=12, fontweight='bold')
            ax.set_xlabel('Number of Agents ($N$)', fontsize=10)
            ax.set_ylabel(title.split('(')[0].strip(), fontsize=10)
            ax.grid(True, linestyle='--', alpha=0.7)
        else:
            ax.text(0.5, 0.5, f"Data for {col} missing", ha='center')

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.savefig('experiment_1_results.png', dpi=300)

def plot_experiment_2(data):
    df = pd.DataFrame.from_dict(data, orient='index')
    df.index = df.index.astype(float)
    df = df.sort_index()

    metrics = {
        'mean_runtime': 'Total Runtime (s)',
        'mean_conflicts': 'Total Conflicts',
        'mean_completed': 'Completed Tasks',
        'mean_avg_latency': 'Avg. Task Completion Time'
    }

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.flatten()
    fig.suptitle('Experiment 2: Performance vs. Task Arrival Rate ($t$)', fontsize=16)
    
    for i, (col, title) in enumerate(metrics.items()):
        ax = axes[i]
        if col in df.columns:
            ax.plot(df.index, df[col], marker='s', linestyle='--', linewidth=2, color='tab:red')
            ax.set_title(title, fontsize=12, fontweight='bold')
            ax.set_xlabel('Task Arrival Rate ($t$ tasks/time)', fontsize=10)
            ax.set_ylabel(title.split('(')[0].strip(), fontsize=10)
            ax.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.savefig('experiment_2_results.png', dpi=300)

def plot_experiment_3(data):
    df = pd.DataFrame.from_dict(data, orient='index')

    metrics = {
        'mean_runtime': 'Total Runtime (s)',
        'mean_conflicts': 'Total Conflicts',
        'mean_makespan': 'Time Steps',
        'mean_avg_latency': 'Avg. Task Completion Time'
    }

    solvers = df.index
    x = np.arange(len(solvers))
    width = 0.5

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes = axes.flatten()
    fig.suptitle('Experiment 3: Solver Comparison (CBS-MAPD vs. Space-Time A*)', fontsize=16)

    colors = ['tab:blue', 'tab:orange']

    for i, (col, title) in enumerate(metrics.items()):
        ax = axes[i]
        if col in df.columns:
            bars = ax.bar(x, df[col], width, color=colors[:len(solvers)])
            
            ax.set_title(title, fontsize=12, fontweight='bold')
            ax.set_xticks(x)
            ax.set_xticklabels(solvers, fontsize=10)
            ax.set_ylabel(title.split('(')[0].strip(), fontsize=10)
            ax.grid(axis='y', linestyle='--', alpha=0.7)

            for bar in bars:
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., 1.05*height,
                        f'{height:.1f}',
                        ha='center', va='bottom', fontsize=9)

            ax.set_ylim(0, max(df[col]) * 1.2)

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.savefig('experiment_3_results.png', dpi=300)

if __name__ == "__main__":
    results = load_results(RESULTS_FILE)
    if results:
        if 'Experiment 1' in results:
            plot_experiment_1(results['Experiment 1'])
        
        if 'Experiment 2' in results:
            plot_experiment_2(results['Experiment 2'])
            
        if 'Experiment 3' in results:
            plot_experiment_3(results['Experiment 3'])
            
        print("\nAll plots generated successfully!")