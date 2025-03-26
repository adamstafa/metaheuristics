#pragma once

#include <memory>
#include <random>
#include <climits>

#include "../problem/solution_manipulator.hpp"
#include "selection.hpp"
#include "rng.hpp"

class BaseInserter
{
public:
    SolutionManipulator& manipulator;

    BaseInserter(SolutionManipulator& manipulator) : manipulator(manipulator) {}

    virtual void insert(std::vector<call_id_t> calls) = 0;
};


class RegretInserter : BaseInserter
{
public:
    std::vector<std::vector<int>> costs; // [call][vehicle]
    std::vector<std::vector<std::vector<call_id_t>>> plans;
    ProblemReimagined& problem;

    RegretInserter(SolutionManipulator& manipulator) : BaseInserter(manipulator), costs(), plans(), problem(manipulator.solution.problem.get())
    {
        costs.push_back({});
        plans.push_back({});
        for (int c = 1; c <= problem.n_calls; c++)
        {
            costs.push_back({});
            plans.push_back({});
            for (int v = 0; v <= problem.n_vehicles; v++)
            {
                costs[c].push_back(INT_MAX);
                plans[c].push_back({});
            }
        }
    };

    void insert(std::vector<call_id_t> calls)
    {
        for (auto call : calls)
        {
            for (vehicle_id_t v = 0; v <= problem.n_vehicles; v++)
            {
                auto x = calculate_insertion_cost(call, v, manipulator); // TODO: rename
                costs[call][v] = x.first;
                plans[call][v] = std::move(x.second);
            }
        }

        while (calls.size() > 0)
        {
            insert_one(calls);
        }
    }

    void insert_one(std::vector<call_id_t>& calls)
    {
        std::vector<int> regret;

        for (auto call : calls)
        {
            std::vector<int> call_costs = costs[call];
            std::sort(call_costs.begin(), call_costs.end());
            regret.push_back(call_costs[1] - call_costs[0]);
        }

        std::vector<std::pair<double, call_id_t>> options;
        for (int i = 0; i < calls.size(); i++)
        {
            options.push_back({-regret[i], calls[i]});
        }

        call_id_t best_call = select_best_geom(options, 0.5).second;

        int best_cost = INT_MAX;
        vehicle_id_t best_vehicle;

        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            if (costs[best_call][v] < best_cost)
            {
                best_vehicle = v;
                best_cost = costs[best_call][v];
            }
        }

        auto& best_plan = plans[best_call][best_vehicle];

        manipulator.set_plan(best_vehicle, best_plan.begin(), best_plan.end());

        calls.erase(std::remove_if(calls.begin(), calls.end(), [best_call](call_id_t c) {
            return abs(c) == abs(best_call);
        }), calls.end());

        for (auto call : calls)
        {
            auto x = calculate_insertion_cost(call, best_vehicle, manipulator); // TODO: rename
            costs[call][best_vehicle] = x.first;
            plans[call][best_vehicle] = std::move(x.second);
        }
    }

    std::pair<int, std::vector<call_id_t>> calculate_insertion_cost(call_id_t call_id, vehicle_id_t vehicle, SolutionManipulator& manipulator)
    {
        if (vehicle == 0)
        {
            auto calls = manipulator.calls[0];
            calls.push_back(call_id);
            calls.push_back(-call_id);
            return {manipulator.solution.problem.get().no_transport_costs[call_id], std::move(calls)};
        }

        auto &og_vs = manipulator.solution.vehicle_solution(vehicle);
        if (!og_vs.problem.get().get_call(call_id).compatible)
        {
            return {INT_MAX, {}};
        }

        std::pair<int, std::vector<call_id_t>> best {INT_MAX, {}};
        VehicleSolution vs{vehicle, manipulator.solution.problem.get().vehicle_problems[vehicle]};
        auto& pickup = vs.problem.get().get_call(call_id);
        auto& delivery = vs.problem.get().get_call(-call_id);
        std::vector<call_id_t> calls = manipulator.calls[vehicle];

        std::vector<int> latest_arrival(calls.size());
        int last = INT_MAX;
        for (int i = calls.size() - 1; i >= 0; i--)
        {
            last = std::min(last, vs.problem.get().get_call(calls[i]).window_high);
            latest_arrival[i] = last;
        }

        for (int i = 0; i <= calls.size(); i++) // i = number of calls before the first insertion place
        {
            if (i > 0)
            {
                vs.remove_many(vs.num_calls() - (i - 1));
                vs.add_call(calls[i - 1]);
            }


            if (vs.plan.back().departure_time > std::min(pickup.window_high, delivery.window_high))
            {
                break;
            }

            vs.add_call(call_id);
        
            if (i < calls.size() && vs.plan.back().departure_time > latest_arrival[i])
            {
                continue;
            }

            for (int j = 0; j <= calls.size() - i; j++) // j = number of calls between the insertion places
            {
                if (j > 0)
                {
                    vs.remove_many(vs.num_calls() - (i + j));
                    vs.add_call(calls[i + j - 1]);
                }

                if (vs.plan.back().departure_time > delivery.window_high)
                {
                    break;
                }

                vs.add_call(-call_id);

                if ((i + j) < calls.size() && vs.plan.back().departure_time > latest_arrival[i + j])
                {
                    continue;
                }

                vs.add_many(calls.begin() + i + j, calls.end());

                if (vs.feasible() && vs.cost() < best.first)
                {
                    // save found solution
                    std::vector<call_id_t> new_calls;
                    for (int i = 1; i <= vs.num_calls(); i++)
                    {
                        new_calls.push_back(vs.plan[i].call);
                    }
                    best = {vs.cost(), std::move(new_calls)};
                }
            }
        }

        // subtract original cost of the vehicle - we care how much more expensive the solution becomes
        best.first -= manipulator.solution.vehicle_solution(vehicle).cost();
        return std::move(best);
    }
};
