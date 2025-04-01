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
        int transfer_time = 0;
        if (i < calls.size() - 1)
        {
            auto call_1 = vs.problem.get().get_call(calls[i]);
            auto call_2 = vs.problem.get().get_call(calls[i + 1]);
            transfer_time = call_1.processing_time + vs.problem.get().travel_time(call_1.id, call_2.id);
        }
        last = std::min(last - transfer_time, vs.problem.get().get_call(calls[i]).window_high);
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

void calculate_best_insertion_option(call_id_t call_id, vehicle_id_t vehicle, SolutionManipulator& manipulator, std::pair<int, std::vector<call_id_t>>& output)
{
    // TODO: if we dont want all options but only the best one, we can just check if the cost is better and check the feasibility later
    if (vehicle == 0)
    {
        output.second.assign(manipulator.calls[0].begin(), manipulator.calls[0].end());
        output.second.push_back(call_id);
        output.second.push_back(-call_id);
        output.first = manipulator.solution.problem.get().no_transport_costs[call_id];
        return;
    }

    auto& og_vs = manipulator.solution.vehicle_solution(vehicle);
    if (!og_vs.problem.get().get_call(call_id).compatible)
    {
        output.first = INT_MAX;
        return;
    }


    VehicleSolution vs{vehicle, manipulator.solution.problem.get().vehicle_problems[vehicle]};
    auto& pickup = vs.problem.get().get_call(call_id);
    auto& delivery = vs.problem.get().get_call(-call_id);
    std::vector<call_id_t>& calls = manipulator.calls[vehicle];
    vs.reserve(calls.size() + 2);

    std::vector<int> latest_arrival(calls.size());
    int last = INT_MAX;
    for (int i = calls.size() - 1; i >= 0; i--)
    {
        int transfer_time = 0;
        if (i < calls.size() - 1)
        {
            auto call_1 = vs.problem.get().get_call(calls[i]);
            auto call_2 = vs.problem.get().get_call(calls[i + 1]);
            transfer_time = call_1.processing_time + vs.problem.get().travel_time(call_1.id, call_2.id);
        }
        last = std::min(last - transfer_time, vs.problem.get().get_call(calls[i]).window_high);
        latest_arrival[i] = last;
    }

    int best_cost = INT_MAX;
    output.first = INT_MAX;
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

            if (vs.feasible() && vs.cost() < best_cost)
            {
                best_cost = vs.cost();
                output.second.resize(vs.num_calls());
                for (int i = 0; i < vs.num_calls(); i++)
                {
                    output.second[i] = vs.plan[i + 1].call;
                }
                output.first = vs.cost() - og_vs.cost();
            }
        }
    }
}

class IterativeInserter : public BaseInserter
{
public:
    // TODO: consider splitting the options into two vectors for faster access to costs only
    std::vector<std::vector<std::pair<int, std::vector<call_id_t>>>> insertion_options; // [call][vehicle]
    ProblemReimagined& problem;

    IterativeInserter(SolutionManipulator& manipulator) : BaseInserter(manipulator), insertion_options(), problem(manipulator.solution.problem.get())
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

    virtual void insert(std::vector<call_id_t> calls)
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

    virtual std::tuple<call_id_t, vehicle_id_t> select_insertion(std::vector<call_id_t>& calls) = 0;

    void insert_one(std::vector<call_id_t>& calls)
    {
        auto insertion = select_insertion(calls);
        auto best_call = std::get<0>(insertion);
        auto best_vehicle = std::get<1>(insertion);
        auto& best_plan  = insertion_options[best_call][best_vehicle].second;

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
        // TODO: improve performance by avoiding copies
        // auto options = calculate_insertion_options(call, vehicle, manipulator);
        // insertion_options[call][vehicle] = *std::min_element(options.begin(), options.end());
        calculate_best_insertion_option(call, vehicle, manipulator, insertion_options[call][vehicle]);
    }
};


class RegretInserter : public IterativeInserter
{
    std::vector<std::pair<double, call_id_t>> options;
    std::vector<int> call_costs;

public:
    RegretInserter(SolutionManipulator& manipulator) : IterativeInserter(manipulator), options()
    {
    };

    virtual std::tuple<call_id_t, vehicle_id_t> select_insertion(std::vector<call_id_t>& calls) override
    {
        options.clear();
        for (auto call : calls)
        {
            call_costs.clear();
            for (auto& option : insertion_options[call])
            {
                call_costs.push_back(option.first);
            }
            std::sort(call_costs.begin(), call_costs.end());
            auto regret = call_costs[1] - call_costs[0];
            options.push_back({ -regret, call });
        }

        call_id_t best_call = select_best_geom(options, 1.0).second;

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
        return { best_call, best_vehicle };
    }
};

class RandomInserter : public BaseInserter
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
