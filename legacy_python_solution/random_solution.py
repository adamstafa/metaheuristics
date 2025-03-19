
from pdp_utils import *
import random
import itertools
import numpy as np
import os
import time


class RandomSolver:
    def __init__(self, data_folder, problem_name, run_id):
        self.problem_name = problem_name
        self.run_id = run_id
        problem_path = f'{data_folder}/{problem_name}'
        self.prob = load_problem(problem_path)
        self.n_calls = self.prob['n_calls']
        self.n_vehicles = self.prob['n_vehicles']
        self.log_path = f'logs/{problem_name}/{run_id}.txt'

        calls = list(range(1, self.n_calls + 1))
        self.init_sol = [ 0 ] * self.n_vehicles + list(itertools.chain(*[ [x, x] for x in calls ]))


    def is_feasible(self, vehicle, schedule):
        fake_sol = [0] * vehicle + schedule + [0] * (self.n_vehicles - vehicle)

        f, c = feasibility_check(fake_sol, self.prob)

        return f


    def generate_random_solution(self):
        vehicle_calls = [[] for _ in range(self.n_vehicles)]
        dummy_calls = []

        # assign calls to vehicles
        for i in range(self.n_calls):
            vehicle_compatible = np.array(self.prob['VesselCargo'][:, i], dtype=bool)
            compatible_vehicles = np.where(vehicle_compatible)[0]
            vehicle = random.choice(compatible_vehicles)
            vehicle_calls[vehicle].append(i + 1)

        # create random schedule
        vehicle_route = [vehicle_calls[i] * 2 for i in range(self.n_vehicles)]
        for route in vehicle_route:
            random.shuffle(route)

        # give infeasible calls to dummy
        for vehicle in range(self.n_vehicles):
            schedule = []
            for c in vehicle_route[vehicle]:
                if c in dummy_calls:
                    continue

                schedule.append(c)
                
                # print(schedule)

                if not self.is_feasible(vehicle, schedule):
                    schedule = [ x for x in schedule if x != c ]
                    dummy_calls += [c, c]

                # if not self.is_feasible(vehicle, schedule):
                #     print(vehicle)
                #     print(schedule)
                #     assert False
                # # TODO: investigate

            vehicle_route[vehicle] = schedule
        
        return list(itertools.chain(*[ s + [0] for s in vehicle_route ])) + dummy_calls

    def generate_solutions(self, n):
        yield self.init_sol
        for _ in range(n):
            yield self.generate_random_solution()

    
    def solve(self):
        generate_attempts = 10000
        start_time = time.time()

        init_objective = cost_function(self.init_sol, self.prob)
        best_objective = init_objective
        best_sol = self.init_sol

        for _ in range(generate_attempts):
            sol = self.generate_random_solution()
            feasibility, c = feasibility_check(sol, self.prob)
            cost = cost_function(sol, self.prob)

            if not feasibility:
                # print(c)
                # print(sol)
                continue
                

            if cost < best_objective:
                best_objective = cost
                best_sol = sol

        end_time = time.time()

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
        def f(i):
            RandomSolver('data', problem, i).solve()
        with Pool(10) as p:
            p.map(f, range(10))