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

class DummyInserter : BaseInserter
{
public:
    DummyInserter(SolutionManipulator& manipulator) : BaseInserter(manipulator)
    {
    };

    void insert(std::vector<call_id_t> calls)
    {
        auto plan = manipulator.calls[0];
        for (auto call : calls)
        {
            plan.push_back(call);
            plan.push_back(-call);
        }
        manipulator.set_plan(0, plan.begin(), plan.end());
    }
};

std::vector<std::pair<int, std::vector<call_id_t>>> calculate_insertion_options(call_id_t call_id, vehicle_id_t vehicle, SolutionManipulator& manipulator)
{
    // TODO: if we dont want all options but only the best one, we can just check if the cost is better and check the feasibility later
    if (vehicle == 0)
    {
        auto calls = manipulator.calls[0];
        calls.push_back(call_id);
        calls.push_back(-call_id);
        return {{manipulator.solution.problem.get().no_transport_costs[call_id], std::move(calls)}};
    }

    auto& og_vs = manipulator.solution.vehicle_solution(vehicle);
    if (!og_vs.problem.get().get_call(call_id).compatible)
    {
        return {{INT_MAX, {}}};
    }

    std::vector<std::pair<int, std::vector<call_id_t>>> options;
    VehicleSolution vs{vehicle, manipulator.solution.problem.get().vehicle_problems[vehicle]};
    auto& pickup = vs.problem.get().get_call(call_id);
    auto& delivery = vs.problem.get().get_call(-call_id);
    std::vector<call_id_t>& calls = manipulator.calls[vehicle];
    vs.reserve(calls.size() + 2);

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

            if (vs.feasible())
            {
                std::vector<call_id_t> calls(vs.num_calls());
                for (int i = 0; i < vs.num_calls(); i++)
                {
                    calls[i] = vs.plan[i + 1].call;
                }
                options.push_back({vs.cost() -og_vs.cost(), std::move(calls)});
            }
        }
    }

    if (options.size() == 0)
    {
        return {{INT_MAX, {}}};
    }
    return std::move(options);
}


class RegretInserter : BaseInserter
{
public:
    std::vector<std::vector<std::pair<int, std::vector<call_id_t>>>> insertion_options; // [call][vehicle]
    ProblemReimagined& problem;

    RegretInserter(SolutionManipulator& manipulator) : BaseInserter(manipulator), insertion_options(), problem(manipulator.solution.problem.get())
    {
        insertion_options.push_back({});
        for (int c = 1; c <= problem.n_calls; c++)
        {
            insertion_options.push_back({});
            for (int v = 0; v <= problem.n_vehicles; v++)
            {
                insertion_options[c].push_back({INT_MAX, {}});
            }
        }
    };

    void insert(std::vector<call_id_t> calls)
    {
        for (auto call : calls)
        {
            for (vehicle_id_t v = 0; v <= problem.n_vehicles; v++)
            {
                update_costs(call, v);
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
            auto call_costs = insertion_options[call];
            std::sort(call_costs.begin(), call_costs.end());
            regret.push_back(call_costs[1].first - call_costs[0].first);
        }

        std::vector<std::pair<double, call_id_t>> options;
        for (int i = 0; i < calls.size(); i++)
        {
            options.push_back({-regret[i], calls[i]});
        }

        call_id_t best_call = select_best_geom(options, 0.8).second;

        int best_cost = INT_MAX;
        vehicle_id_t best_vehicle;

        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            if (insertion_options[best_call][v].first < best_cost)
            {
                best_vehicle = v;
                best_cost = insertion_options[best_call][v].first;
            }
        }

        auto& best_plan = insertion_options[best_call][best_vehicle].second;

        manipulator.set_plan(best_vehicle, best_plan.begin(), best_plan.end());

        calls.erase(std::remove_if(calls.begin(), calls.end(), [best_call](call_id_t c) {
            return abs(c) == abs(best_call);
        }), calls.end());

        for (auto call : calls)
        {
            update_costs(call, best_vehicle);
        }
    }

    void update_costs(call_id_t call, vehicle_id_t vehicle)
    {
        auto options = calculate_insertion_options(call, vehicle, manipulator);
        insertion_options[call][vehicle] = *std::min_element(options.begin(), options.end());
    }
};

class RandomInserter : BaseInserter
{
public:
    ProblemReimagined& problem;

    RandomInserter(SolutionManipulator& manipulator) : BaseInserter(manipulator), problem(manipulator.solution.problem.get())
    {
    };

    void insert(std::vector<call_id_t> calls)
    {
        std::shuffle(calls.begin(), calls.end(), gen);
        for (auto call : calls)
        {
            std::vector<std::pair<int, std::vector<call_id_t>>> feasible_options; // pairs <vehicle, calls>
            for (vehicle_id_t v = 0; v <= problem.n_vehicles; v++)
            {
                auto options = calculate_insertion_options(call, v, manipulator);
                for (int i = 0; i < options.size(); i++)
                {
                    if (options[i].first != INT_MAX)
                    {
                        feasible_options.push_back({v, std::move(options[i].second)});
                    }
                }
            }
            auto& selected_option = feasible_options[gen() % feasible_options.size()];
            manipulator.set_plan(selected_option.first, selected_option.second.begin(), selected_option.second.end());
        }
    }
};
