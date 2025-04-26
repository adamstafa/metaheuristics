#pragma once

#include <memory>
#include <random>
#include <climits>

#include "../problem/solution_manipulator.hpp"
#include "selection.hpp"
#include "rng.hpp"
#include "hungarian.hpp"

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


class IterativeInserter : public BaseInserter
{
public:
    // TODO: consider splitting the options into two vectors for faster access to costs only
    std::vector<std::vector<std::pair<int, std::vector<call_id_t>>>> insertion_options; // [call][vehicle]
    std::vector<VehicleSolution> vehicle_solutions; // [vehicle]
    ProblemReimagined& problem;

    IterativeInserter(SolutionManipulator& manipulator) : BaseInserter(manipulator), insertion_options(), vehicle_solutions(), problem(manipulator.solution.problem.get())
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
        
        for (int vehicle = 1; vehicle <= problem.n_vehicles; vehicle++)
        {
            vehicle_solutions.push_back({vehicle, manipulator.solution.problem.get().vehicle_problems[vehicle]});
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

        VehicleSolution& vs = vehicle_solutions[vehicle - 1];
        vs.remove_many(vs.num_calls());

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

        // TODO: explore insertion options
        // selecting always the best leads to low robustness in smaller instances
        // call_id_t best_call = select_best(options).second;
        call_id_t best_call = select_best_geom(options, 0.9).second;
        // call_id_t best_call = select_proportionally(options).second;

        std::vector<std::pair<double, vehicle_id_t>> vehicle_costs;
        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            vehicle_costs.push_back({insertion_options[best_call][v].first, v});
        }
        
        vehicle_id_t best_vehicle = select_best(vehicle_costs).second;
        // call_id_t best_call = select_best_geom(options, 0.9).second;
        // vehicle_id_t best_vehicle = select_proportionally(vehicle_costs).second;

        return { best_call, best_vehicle };
    }
};

class GreedyInserter : public IterativeInserter
{
    std::vector<std::pair<double, call_id_t>> options;
    std::vector<int> call_costs;

public:
    GreedyInserter(SolutionManipulator& manipulator) : IterativeInserter(manipulator), options()
    {
    };

    virtual std::tuple<call_id_t, vehicle_id_t> select_insertion(std::vector<call_id_t>& calls) override
    {
        call_id_t best_call = calls[gen() % calls.size()];

        std::vector<std::pair<double, vehicle_id_t>> vehicle_costs;
        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            vehicle_costs.push_back({insertion_options[best_call][v].first, v});
        }
        vehicle_id_t best_vehicle = select_best(vehicle_costs).second;

        return { best_call, best_vehicle };
    }
};

class MatchingInserter : public IterativeInserter
{
public:
    MatchingInserter(SolutionManipulator& manipulator) : IterativeInserter(manipulator)
    {
    };

    virtual std::tuple<call_id_t, vehicle_id_t> select_insertion(std::vector<call_id_t>& calls) override
    {
        int lhs_vertices = calls.size();
        int rhs_vertices = problem.n_vehicles + lhs_vertices; // we need dummy vehicle for each call
        auto get_vehicle = [&] (int vehicle_vertex)
        {
            if (vehicle_vertex >= problem.n_vehicles)
                return 0;
            return vehicle_vertex + 1;
        };
        auto edge_cost = [&] (int call, int vehicle)
        {
            return this->insertion_options[calls[call]][get_vehicle(vehicle)].first;
        };
        auto matching = munkres_algorithm<long>(lhs_vertices, rhs_vertices, edge_cost);
        

        int cheapest_count = 0;
        for (int i = 0; i < calls.size(); i++)
        {
            int cost = INT_MAX;
            for (vehicle_id_t v = 0; v <= problem.n_vehicles; v++)
            {
                cost = std::min(cost, insertion_options[calls[i]][v].first);
            }
            
            auto edge = matching[i];
            if (edge_cost(edge.first, edge.second) == cost)
            {
                cheapest_count++;
                return {calls[edge.first], get_vehicle(edge.second)};
            }
        }

        // std::cout << "matching size: " << matching.size() << ", perfect: " << cheapest_count << std::endl;


        bool exists_non_dummy = false;
        for (auto& edge : matching)
        {
            if (get_vehicle(edge.second) != 0)
                exists_non_dummy = true;
        }


        // number of cheaper options heuristic
        std::vector<int> cheaper_options;
        for (int i = 0; i < calls.size(); i++)
        {
            auto it = std::find_if(matching.begin(), matching.end(), [i](const std::pair<int, int>& edge) {
                return edge.first == i;
            });
            auto edge = *it;

            auto cost = edge_cost(edge.first, edge.second);
            int count = 0;
            for (vehicle_id_t v = 0; v <= problem.n_vehicles; v++)
            {
                if (insertion_options[calls[i]][v].first <= cost)
                    count++;
            }
            cheaper_options.push_back(count);
        }
        
        auto it = std::min_element(cheaper_options.begin(), cheaper_options.end());
        int call_index = std::distance(cheaper_options.begin(), it);
        auto edge = *std::find_if(matching.begin(), matching.end(), [call_index](auto& e){ return e.first == call_index; });



        // random edge
        edge = matching[gen() % matching.size()];



        auto call = calls[edge.first];
        auto vehicle = get_vehicle(edge.second);
        return { call, vehicle };
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
