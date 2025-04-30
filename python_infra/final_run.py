
from pdp_utils import *
import itertools
import os
import time


class Solver:
    def __init__(self, data_folder, problem_name, duration=10):
        self.problem_name = problem_name
        self.problem_path = f'{data_folder}/{problem_name}'
        self.prob = load_problem(self.problem_path)
        self.n_calls = self.prob['n_calls']
        self.n_vehicles = self.prob['n_vehicles']
        self.duration = duration
        
        self.log_path = f'final_run/{problem_name}'
        self.solver_path = f'build/optimize'

        calls = list(range(1, self.n_calls + 1))
        self.init_sol = [ 0 ] * self.n_vehicles + list(itertools.chain(*[ [x, x] for x in calls ]))

    
    def check_validity(self, sol):
        vehicle_count = sol.count(0)
        call_counts = {i: sol.count(i) for i in range(1, self.n_calls + 1)}

        assert vehicle_count == self.n_vehicles
        for call, count in call_counts.items():
            assert count == 2

    def solve(self):
        proc = os.popen(f'{self.solver_path} {self.problem_path} {self.duration}')
        output = proc.read().strip()
        sol = [ int(x) for x in output[1:-1].split(',') ]
        return sol

    def run(self):
        start_time = time.time()

        init_objective = cost_function(self.init_sol, self.prob)

        best_sol = self.solve()
        best_objective = cost_function(best_sol, self.prob)

        end_time = time.time()

        assert feasibility_check(best_sol, self.prob)[0]
        self.check_validity(best_sol)

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        with open(self.log_path, 'w') as f:
            # print(best_objective, file=f)
            print(best_sol, file=f)
            # print(end_time - start_time, file=f)
            # print(init_objective, file=f)
            # print(100 * (1 - best_objective / init_objective), file=f)

        return {
            'objective': best_objective,
            'solution': best_sol,
            'problem': self.problem_name,
        }


from multiprocessing import Pool

problems = [
    'Call_7_Vehicle_3.txt',
    'Call_18_Vehicle_5.txt',
    'Call_35_Vehicle_7.txt',
    'Call_80_Vehicle_20.txt',
    'Call_130_Vehicle_40.txt',
    # 'Call_300_Vehicle_90.txt'
]

durations = {
    'Call_7_Vehicle_3.txt': 5,
    'Call_18_Vehicle_5.txt': 10,
    'Call_35_Vehicle_7.txt': 60,
    'Call_80_Vehicle_20.txt': 600,
    'Call_130_Vehicle_40.txt': 600,
    # 'Call_300_Vehicle_90.txt': 600
}

data_dir = 'data'

if __name__ == '__main__':
    def f(problem):
        return Solver(data_dir, problem, durations[problem]).run()
    with Pool(len(problems)) as p:
        results = p.map(f, problems)
        print()
        print("Final Results")
        for result in results:
            print(f"{result['problem']:<{max(map(len, problems))}}   {result['objective']:<10}")
