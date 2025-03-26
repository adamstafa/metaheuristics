// void compute_best_insertion_place(VehicleSolution& vs, std::vector<call_id_t>& remaining, std::pair<int, std::vector<call_id_t>>& current, std::pair<int, std::vector<call_id_t>>& best)
// {
//     auto& problem = vs.problem.get();
//     // skip if infeasible
//     if (!vs.feasible())
//     {
//         return ;
//     }
//     // skip if solution can't be finished
//     for (auto rem : remaining)
//     {
//         if (problem.get_call(rem).window_high < vs.plan.back().departure_time)
//         {
//             return;
//         }
//     }

//     if (remaining.empty() && vs.feasible() && vs.cost() < best.first)
//     {
//         best = current;
//     }

//     for (auto c : remaining)
//     {
//         // not picked up yet
//         if (c < 0 && std::find(remaining.begin(), remaining.end(), -c) != remaining.end())
//         {
//             continue;
//         }

//         current.second.push_back(c);
        
//         std::vector<call_id_t> new_remaining;
//         for (auto rem : remaining)
//         {
//             if (rem != c)
//             {
//             new_remaining.push_back(rem);
//             }
//         }

//         vs.add_call(c);
//         compute_best_insertion_place(vs, new_remaining, current, best);
//         vs.remove_call();

//         current.second.pop_back();
//     }
// }

// std::pair<int, std::vector<call_id_t>> compute_best_permutation(VehicleSolution& vehicle_solution_ref)
// {
//     VehicleSolution vs{vehicle_solution_ref.vehicle, vehicle_solution_ref.problem};
//     std::vector<call_id_t> calls;
//     for (int i = 0; i < vehicle_solution_ref.num_calls(); i++)
//     {
//         calls.push_back(vehicle_solution_ref.plan[i+1].call);
//     }
    
//     std::pair<int, std::vector<call_id_t>> current, best;
//     best.first = INT_MAX;

//     compute_best_insertion_place(vs, calls, current, best);

//     return best;
// }

// class BestPermutationOperator : public BaseOperator
// {
// public:
//     BestPermutationOperator(SolutionManipulator& manipulator) : BaseOperator(manipulator) {}

//     void apply() override
//     {
//         for (auto& vs : manipulator.solution.vehicle_solutions)
//         {
//             auto best_permutation = compute_best_permutation(*vs).second;
//             manipulator.set_plan(vs->vehicle, best_permutation);
//         }
//     }
// };


// goal: write greedy reinsert algorithm
// input: set of several calls
// for each call...
//   go through all compatible vehicles
//   try to insert the calls on all possible places (there will be only a few feasible places, do fast checks)
//   mark the best places for each call
//
// then greedily insert the calls one by one in randomized order
// 

// std::pair<int, std::vector<call_id_t>> calculate_insertion_cost_all_permutations(call_id_t call, vehicle_id_t vehicle, SolutionManipulator manipulator)
// {
//     if (vehicle == 0)
//     {
//         auto calls = manipulator.calls[0];
//         calls.push_back(call);
//         calls.push_back(call);
//         return {manipulator.solution.problem.get().no_transport_costs[call], calls};
//     }

//     VehicleSolution vs{vehicle, manipulator.solution.problem.get().vehicle_problems[vehicle]};

//     std::vector<call_id_t> calls;
//     for (int i = 0; i < manipulator.solution.vehicle_solution(vehicle).num_calls(); i++)
//     {
//         calls.push_back(manipulator.solution.vehicle_solution(vehicle).plan[i+1].call);
//     }

//     // these are signed calls on level of VehicleSolution
//     calls.push_back(call);
//     calls.push_back(-call);
    
//     std::pair<int, std::vector<call_id_t>> current, best;
//     best.first = INT_MAX;

//     compute_best_insertion_place(vs, calls, current, best);
//     int cost = best.first - manipulator.solution.vehicle_solution(vehicle).cost();

//     return {cost, best.second};
// }

// void greedy_insert(std::vector<call_id_t> calls, SolutionManipulator &manipulator)
// {
//     auto& problem = manipulator.solution.problem.get();
 
//     std::vector<std::vector<std::pair<int, std::vector<call_id_t>>>> costs; // costs[c][v] is the cost of inserting call c into vehicle v
//     costs.push_back({});
//     for (int c = 1; c <= problem.n_calls; c++)
//     {
//         costs.push_back({});
        
//         if (std::find(calls.begin(), calls.end(), c) == calls.end())
//         {
//             continue;
//         }
//         // TODO: space wasted in the array

//         for (int v = 0; v <= problem.n_vehicles; v++)
//         {
//             costs[c].push_back(calculate_insertion_cost_all_permutations(c, v, manipulator));
//         }
//     }

//     std::shuffle(calls.begin(), calls.end(), gen);

//     while (calls.size() > 0)
//     {
//         call_id_t call = calls.back();
//         calls.pop_back();

//         std::pair<int, std::vector<call_id_t>> best_plan;
//         best_plan.first = INT_MAX;
//         vehicle_id_t best_vehicle = -1;

//         for (int v = 0; v <= problem.n_vehicles; v++)
//         {
//             if (costs[call][v] < best_plan)
//             {
//                 best_plan = costs[call][v];
//                 best_vehicle = v;
//             }
//         }

//         manipulator.set_plan(best_vehicle, best_plan.second.begin(), best_plan.second.end());

//         for (auto c : calls)
//         {
//             costs[c][best_vehicle] = calculate_insertion_cost_all_permutations(c, best_vehicle, manipulator);
//         }
//     }
// }
