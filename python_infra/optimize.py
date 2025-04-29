import optuna
import os

problems = [
    'Call_7_Vehicle_3.txt',
    'Call_18_Vehicle_5.txt',
    'Call_35_Vehicle_7.txt',
    'Call_80_Vehicle_20.txt',
    'Call_130_Vehicle_40.txt',
    # 'Call_300_Vehicle_90.txt'
]

problems_path = 'data/'

program_path = 'build/optimize'

def run_once(problem, w1, w2, w3, w4):
    problem_path = os.path.join(problems_path, problem)
    proc = os.popen(f'{program_path} {problem_path} {w1} {w2} {w3} {w4}')
    result = proc.read().strip()
    return float(result)

def run_multiple(problem, w1, w2, w3, w4, n=10):
    results = []
    for _ in range(n):
        result = run_once(problem, w1, w2, w3, w4)
        results.append(result)
    return sum(results) / len(results)

def evaluate_problems(w1, w2, w3, w4):
    results = []
    for problem in problems:
        result = run_multiple(problem, w1, w2, w3, w4)
        results.append(result)
    return sum(results)

def objective(trial):
    w1 = trial.suggest_float('w1', 0.0, 1.0)
    w2 = trial.suggest_float('w2', 0.0, 1.0)
    w3 = trial.suggest_float('w3', 0.0, 1.0)
    w4 = 1.0 - w1 - w2 - w3

    if w4 < 0:
        return float('inf')

    return evaluate_problems(w1, w2, w3, w4)
    

if __name__ == "__main__":
    study = optuna.create_study(direction='minimize', sampler=optuna.samplers.RandomSampler())
    study.optimize(objective, n_jobs=8, timeout=3600*7)
    
    
    fig = optuna.visualization.plot_contour(study)
    fig.write_html('opt_result.html')
    fig.write_image("opt_result.png")

    fig.show()

    print(f"Best trial: {study.best_trial.value}")
    print(f"Best parameters: {study.best_trial.params}")