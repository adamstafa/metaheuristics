
from pdp_utils import *
import random
import itertools
import numpy as np
import os
import time


class Solver:
    def __init__(self, data_folder, problem_name, run_id, algorithm):
        self.problem_name = problem_name
        self.run_id = run_id
        problem_path = f'{data_folder}/{problem_name}'
        self.prob = load_problem(problem_path)
        self.n_calls = self.prob['n_calls']
        self.n_vehicles = self.prob['n_vehicles']
        
        self.log_path = f'logs/{algorithm}/{problem_name}/{run_id}.txt'

        calls = list(range(1, self.n_calls + 1))
        self.init_sol = [ 0 ] * self.n_vehicles + list(itertools.chain(*[ [x, x] for x in calls ]))


    def is_feasible(self, vehicle, schedule):
        fake_sol = [0] * vehicle + schedule + [0] * (self.n_vehicles - vehicle)

        f, c = feasibility_check(fake_sol, self.prob)

        return f


    def apply_operator(self, sol):
        remove_from_dummy_prob = 0.3
        if sol[-1] != 0 and random.random() < remove_from_dummy_prob:
            last_zero_index = len(sol) - 1 - sol[::-1].index(0)
            call_to_remove = random.choice(sol[last_zero_index + 1:])
        else:
            call_to_remove = random.randint(1, self.n_calls)

        # sample compatible vehicle
        vehicle_compatible = np.array(self.prob['VesselCargo'][:, call_to_remove - 1], dtype=bool)
        compatible_vehicles = np.where(vehicle_compatible)[0]
        new_vehicle = random.choice(compatible_vehicles)
        
        # Split the array sol on places where there is 0
        routes = [[]]
        for x in sol:
            if x == 0:
                routes.append([])
            elif x == call_to_remove:
                continue
            else:
                routes[-1].append(x)
        
        pos1 = random.randint(0, len(routes[new_vehicle]))
        pos2 = random.randint(0, len(routes[new_vehicle]))
        routes[new_vehicle].insert(pos1, call_to_remove)
        routes[new_vehicle].insert(pos2, call_to_remove)

        new_sol = list(itertools.chain(*[ s + [0] for s in routes ]))[:-1]
        return new_sol
    
    def apply_operator_repeat(self, sol, attempts=15):
        for _ in range(attempts):
            new_sol = self.apply_operator(sol)
            if feasibility_check(new_sol, self.prob)[0]:
                return new_sol
            
        # print("operator failed")
        return sol

    
    def check_validity(self, sol):
        vehicle_count = sol.count(0)
        call_counts = {i: sol.count(i) for i in range(1, self.n_calls + 1)}

        assert vehicle_count == self.n_vehicles
        for call, count in call_counts.items():
            assert count == 2

    def solve_simulated_annealing(self):
        total_steps = 10000
        warmup_steps = 100

        start_time = time.time()

        init_objective = cost_function(self.init_sol, self.prob)
        best_objective = init_objective
        best_sol = self.init_sol

        final_temp = 0.1
        incumbent = best_sol
        incumbent_cost = init_objective
        deltas = []

        for _ in range(warmup_steps):
            new_sol = self.apply_operator_repeat(incumbent)
            feasibility, c = feasibility_check(new_sol, self.prob)

            if not feasibility:
                continue

            
            new_cost = cost_function(new_sol, self.prob)
            delta = new_cost - incumbent_cost
            if delta < 0:
                incumbent = new_sol
                incumbent_cost = new_cost
            else:
                if random.random() < 0.8:
                    incumbent = new_sol
                    incumbent_cost = new_cost
                deltas.append(delta)
        
        delta_avg = sum(deltas) / len(deltas)

        min_init_temp = 100       
        init_temp = max(- delta_avg / np.log(0.8), min_init_temp)
        alpha = (final_temp / init_temp) ** (1 / (total_steps - warmup_steps))
        temp = init_temp

        for _ in range(total_steps - warmup_steps):
            new_sol = self.apply_operator_repeat(incumbent)
            feasibility, c = feasibility_check(new_sol, self.prob)
            if not feasibility:
                continue

            new_cost = cost_function(new_sol, self.prob)
            delta = new_cost - incumbent_cost


            if delta < 0:
                incumbent = new_sol
                incumbent_cost = new_cost

                if incumbent_cost < best_objective:
                    best_objective = incumbent_cost
                    best_sol = incumbent
            elif random.random() < np.exp(-delta / temp):
                incumbent = new_sol
                incumbent_cost = new_cost
            
            temp *= alpha

        end_time = time.time()

        assert feasibility_check(best_sol, self.prob)[0]
        self.check_validity(best_sol)

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        with open(self.log_path, 'w') as f:
            print(best_objective, file=f)
            print(best_sol, file=f)
            print(end_time - start_time, file=f)
            print(init_objective, file=f)
            print(100 * (1 - best_objective / init_objective), file=f)


    def solve_local_search(self):
        generate_attempts = 10000
        start_time = time.time()

        init_objective = cost_function(self.init_sol, self.prob)
        best_objective = init_objective
        best_sol = self.init_sol

        for _ in range(generate_attempts):
            new_sol = self.apply_operator_repeat(best_sol)
            feasibility, c = feasibility_check(new_sol, self.prob)
            cost = cost_function(new_sol, self.prob)

            if not feasibility:
                continue
                

            if cost < best_objective:
                best_objective = cost
                best_sol = new_sol

        end_time = time.time()

        assert feasibility_check(best_sol, self.prob)[0]
        self.check_validity(best_sol)

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        with open(self.log_path, 'w') as f:
            print(best_objective, file=f)
            print(best_sol, file=f)
            print(end_time - start_time, file=f)
            print(init_objective, file=f)
            print(100 * (1 - best_objective / init_objective), file=f)
            

from multiprocessing import Pool

problems = [
    'Call_7_Vehicle_3.txt',
    'Call_18_Vehicle_5.txt',
    'Call_35_Vehicle_7.txt',
    'Call_80_Vehicle_20.txt',
    'Call_130_Vehicle_40.txt',
    'Call_300_Vehicle_90.txt'
]

if __name__ == '__main__':
    for problem in problems:
        print(f'{problem} - Local Search')
        def f(i):
            Solver('data', problem, i, 'local_search').solve_local_search()
        with Pool(10) as p:
            p.map(f, range(10))

if __name__ == '__main__':
    for problem in problems:
        print(f'{problem} - Simulated Annealing')
        def f(i):
            Solver('data', problem, i, 'simulated_annealing').solve_simulated_annealing()
        with Pool(10) as p:
            p.map(f, range(10))
