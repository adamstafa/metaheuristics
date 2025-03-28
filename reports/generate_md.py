import pandas as pd
import textwrap

def break_long_lines(text, width=90):
    """
    Breaks long lines into shorter ones of specified width while keeping words intact.
    
    :param text: The input string to be wrapped
    :param width: The maximum width of each line (default is 80 characters)
    :return: A string with wrapped lines
    """
    return "\n".join(textwrap.wrap(text, width))

problems = [
    'Call_7_Vehicle_3',
    'Call_18_Vehicle_5',
    'Call_35_Vehicle_7',
    'Call_80_Vehicle_20',
    'Call_130_Vehicle_40',
    'Call_300_Vehicle_90'
]

algorithms = {
    'random_search': 'Random Search',
    'local_search': 'Local Search -- 1-insert',
    'simulated_annealing': 'Simulated Annealing -- 1-insert',
    'simulated_annealing_operators_untuned': 'SA -- new operators (equal weights)',
    'simulated_annealing_operators_tuned': 'SA -- new operators (tuned weights)',
    'alns': 'Adaptive Algorithm',
}

new_algos = {'alns'}

def parse_run(problem, algo):
    runs = []
    for i in range(10):
        with open(f"../logs/{algo}/{problem}.txt/{i}.txt", "r") as f:
            content = f.readlines()
            run = {
                'objective': float(content[0]),
                'solution': content[1].strip(),
                'time': float(content[2]),
                'initial_objective': float(content[3])
            }
            runs.append(run)
    return pd.DataFrame(runs)

def process_problem_algo(problem, algo):
    df = parse_run(problem, algo)
    best_objective = int(df["objective"].min())
    avg_objective = int(df["objective"].mean())
    best_solution = df.loc[df["objective"].idxmin(), "solution" ]
    run_time = df["time"].mean()
    initial_objective = df["initial_objective"].mean()
    improvement = 100 * (initial_objective - best_objective) / initial_objective
    return avg_objective, best_objective, best_solution, run_time, improvement


def process_problem(problem, f):
    best_pool = []
    assert new_algos.issubset(algorithms.keys())

    print(f"## {problem}" , file=f)

    print( "|               | Average objective | Best objective   | Improvement (%)   | Running time (s) |", file=f)
    print( "|       -       |                -: |               -: |                -: |               -: |", file=f)
    for algo, algo_name in algorithms.items():
        avg_objective, best_objective, best_solution, run_time, improvement = process_problem_algo(problem, algo)
        print(f"| {algo_name} | {avg_objective} | {best_objective} | {improvement:.2f} | {run_time:.2f} |", file=f)
        if algo in new_algos:
            best_pool.append((best_objective, best_solution))
    print(file=f)

    # print("Solution:", file=f)
    print("```python", file=f)
    best_solution = min(best_pool, key=lambda x: x[0])[1]
    print(break_long_lines(best_solution), file=f)
    print("```", file=f)
    print(file=f)

with open("output/output.md", "w") as f:
    print("# INF273 - Assignment 5", file=f)
    print("Adam Štafa", file=f)
    print(file=f)
    for problem in problems:
        process_problem(problem, f)
