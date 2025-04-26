#pragma once

#include <memory>
#include <random>
#include <climits>

#include "../problem/solution_manipulator.hpp"
#include "insert.hpp"
#include "rng.hpp"

class BaseRemover
{
public:
    SolutionManipulator& manipulator;

    BaseRemover(SolutionManipulator& manipulator)
        : manipulator(manipulator) {}

    virtual std::vector<call_id_t> remove(int num_elements) = 0;
};

class RandomRemover : BaseRemover
{
public:
    RandomRemover(SolutionManipulator& manipulator) : BaseRemover(manipulator) {}

    std::vector<call_id_t> remove(int num_elements) override
    {
        std::vector<call_id_t> removed_calls;

        while (removed_calls.size() < num_elements)
        {
            int removed_call = gen() %  manipulator.solution.problem.get().n_calls + 1;
            if (std::find(removed_calls.begin(), removed_calls.end(), abs(removed_call)) != removed_calls.end())
                continue;

            int vehicle = manipulator.get_vehicle_for_call(removed_call);
            auto calls = manipulator.calls[vehicle];
            calls.erase(std::remove_if(calls.begin(), calls.end(), [removed_call](call_id_t c) {
                return abs(c) == abs(removed_call);
            }), calls.end());
            removed_calls.push_back(abs(removed_call));
            manipulator.set_plan(vehicle, calls.begin(), calls.end());
        }
    
        return removed_calls;
    }
};

class SimilarVehiclesRemover : BaseRemover
{    
public:
    inline static double cargo_weight;
    inline static double distance_weight;
    inline static double time_weight;
    inline static double compatibility_weight;

    std::vector<std::vector<double>> similarities;

    SimilarVehiclesRemover(SolutionManipulator& manipulator) : BaseRemover(manipulator), similarities()
    {
        similarities.push_back({});
        for (call_id_t call_1 = 1; call_1 <= manipulator.solution.problem.get().n_calls; call_1++)
        {
            similarities.push_back({});
            similarities[call_1].push_back({});
            for (call_id_t call_2 = 1; call_2 <= manipulator.solution.problem.get().n_calls; call_2++)
            {
                similarities[call_1].push_back(similarity_score(call_1, call_2));
            }
        }
    }

    double average_distance(call_id_t call_id, std::vector<call_id_t>& calls)
    {
        double sum = 0;
        for (auto call : calls)
        {
            sum += similarities[abs(call_id)][abs(call)];
        }
        return sum / calls.size();
    }

    std::vector<call_id_t> remove(int num_elements)
    {
        call_id_t first_call = rand() % manipulator.solution.problem.get().n_calls + 1;
        std::vector<call_id_t> removed_calls;
        removed_calls.push_back(first_call);

        while (removed_calls.size() < num_elements)
        {
            std::vector<std::pair<double, call_id_t>> distances;
            for (call_id_t call = 1; call <= manipulator.solution.problem.get().n_calls; call++)
            {
                if (std::find(removed_calls.begin(), removed_calls.end(), abs(call)) != removed_calls.end())
                    continue;

                distances.push_back({average_distance(call, removed_calls), call});
            }
            call_id_t call = select_best_geom(distances, 0.2).second;
            // call_id_t call = select_proportionally(distances).second;
            removed_calls.push_back(abs(call));
        }

        for (auto removed_call : removed_calls)
        {
            vehicle_id_t vehicle = manipulator.get_vehicle_for_call(removed_call);
            auto calls = manipulator.calls[vehicle];
            calls.erase(std::remove_if(calls.begin(), calls.end(), [removed_call](call_id_t c) {
                return abs(c) == abs(removed_call);
            }), calls.end());
            manipulator.set_plan(vehicle, calls.begin(), calls.end());
        }

        return removed_calls;
    }
    
    double similarity_score(call_id_t call_1_id, call_id_t call_2_id)
    {
        auto& solution = manipulator.solution;
        auto& problem = solution.problem.get();
        auto& vp = problem.vehicle_problems[1];
        auto call_1_pickup = vp.get_call(call_1_id);
        auto call_1_delivery = vp.get_call(-call_1_id);
        auto call_2_pickup = vp.get_call(call_2_id);
        auto call_2_delivery = vp.get_call(-call_2_id);
        
        int max_travel_time = *std::max_element(vp.travel_times.data.begin(), vp.travel_times.data.end());
        double pickup_intersection = std::max(0, std::min(call_1_pickup.window_high, call_2_pickup.window_high) - std::max(call_1_pickup.window_low, call_2_pickup.window_low));
        double pickup_score = pickup_intersection / (std::max(call_1_pickup.window_high - call_1_pickup.window_low, call_2_pickup.window_high - call_2_pickup.window_low));
        double delivery_intersection = std::max(0, std::min(call_1_delivery.window_high, call_2_delivery.window_high) - std::max(call_1_delivery.window_low, call_2_delivery.window_low));
        double delivery_score = delivery_intersection / (std::max(call_1_delivery.window_high - call_1_delivery.window_low, call_2_delivery.window_high - call_2_delivery.window_low));
        int compatible_intersection_size = 0;
        int compatible_union_size = 0;
        for (vehicle_id_t v = 1; v <= problem.n_vehicles; v++)
        {
            auto& vp = problem.vehicle_problems[v];
            bool c1 = vp.get_call(call_1_id).compatible;
            bool c2 = vp.get_call(call_2_id).compatible;
            compatible_intersection_size += c1 && c2;
            compatible_union_size += c1 || c2;
        }
        assert(compatible_union_size > 0);

        auto cargo_similarity = std::abs(call_1_pickup.size - call_2_pickup.size) / (double) std::max(call_1_pickup.size, call_2_pickup.size);
        auto distance_similarity = (vp.travel_time(call_1_id, call_2_id) + vp.travel_time(- call_1_id, - call_2_id)) / (double) (2 * max_travel_time);
        auto time_similarity = 1 - (pickup_score + delivery_score) / 2;
        auto compatibility_similarity = 1 - compatible_intersection_size / (double) compatible_union_size;

        return (SimilarVehiclesRemover::cargo_weight * cargo_similarity + SimilarVehiclesRemover::distance_weight * distance_similarity + SimilarVehiclesRemover::time_weight * time_similarity + SimilarVehiclesRemover::compatibility_weight * compatibility_similarity) / 4.0;
    }
};

class FullVehiclesRemover : BaseRemover
{
public:
    FullVehiclesRemover(SolutionManipulator& manipulator) : BaseRemover(manipulator) {}

    std::vector<call_id_t> remove(int num_elements)
    {
        std::vector<call_id_t> removed_calls;

        call_id_t first_call = rand() % manipulator.solution.problem.get().n_calls + 1;
        std::vector<std::pair<double, call_id_t>> distances; // rename to scores?
        for (call_id_t call = 1; call <= manipulator.solution.problem.get().n_calls; call++)
        {
            auto vehicle = manipulator.get_vehicle_for_call(call);
            int num_calls_in_vehicle = manipulator.calls[vehicle].size();
            distances.push_back({-num_calls_in_vehicle, call});
        }

        while (removed_calls.size() < num_elements)
        {
            call_id_t call = select_best_geom(distances, 0.05).second;
            // call_id_t call = select_proportionally(distances).second;
            distances.erase(std::remove_if(distances.begin(), distances.end(), [call](const std::pair<int, call_id_t>& p) {
                return p.second == call || p.second == -call;
            }), distances.end());
            removed_calls.push_back(abs(call));
        }

        for (auto removed_call : removed_calls)
        {
            vehicle_id_t vehicle = manipulator.get_vehicle_for_call(removed_call);
            auto calls = manipulator.calls[vehicle];
            calls.erase(std::remove_if(calls.begin(), calls.end(), [removed_call](call_id_t c) {
                return abs(c) == abs(removed_call);
            }), calls.end());
            manipulator.set_plan(vehicle, calls.begin(), calls.end());
        }

        return removed_calls;
    }
};

class ExpensiveCallRemover : BaseRemover
{
public:
    ExpensiveCallRemover(SolutionManipulator& manipulator) : BaseRemover(manipulator) {}

    std::vector<call_id_t> remove(int num_elements)
    {
        std::vector<int> cost_diff(manipulator.solution.problem.get().n_calls + 1, 0); // cost difference after removing the call

        for (call_id_t call = 1; call <= manipulator.solution.problem.get().n_calls; call++)
        {
            auto vehicle = manipulator.get_vehicle_for_call(call);
            if (vehicle == 0)
            {
                cost_diff[call] = - manipulator.solution.problem.get().no_transport_costs[call];
                continue;
            }

            auto& vs = manipulator.solution.vehicle_solution(vehicle);
            auto& vp = vs.problem.get();

            int pickup_index;
            int delivery_index;

            for (int i = 0; i < vs.plan.size(); i++)
            {
                if (vs.plan[i].call == call)
                {
                    pickup_index = i;
                }
                else if (vs.plan[i].call == -call)
                {
                    delivery_index = i;
                }
            }

            int before_pickup = vs.plan[pickup_index - 1].call;
            int after_pickup = vs.plan[pickup_index + 1].call;
            int before_delivery = vs.plan[delivery_index - 1].call;
            int after_delivery = delivery_index + 1 < vs.plan.size() ? vs.plan[delivery_index + 1].call : vs.plan[delivery_index].call;

            int cost_to_pickup = vp.travel_cost(before_pickup, call);
            int cost_from_pickup = vp.travel_cost(call, after_pickup);
            int cost_to_delivery = vp.travel_cost(before_delivery, -call);
            int cost_from_delivery = vp.travel_cost(-call, after_delivery);

            if (delivery_index == pickup_index + 1)
            {
                cost_diff[call] = - cost_to_pickup - cost_from_delivery + vp.travel_cost(before_pickup, after_delivery);
            }
            else
            {
                cost_diff[call] = - cost_to_pickup - cost_from_pickup + vp.travel_cost(before_pickup, after_pickup) - cost_to_delivery - cost_from_delivery + vp.travel_cost(before_delivery, after_delivery);
            }
        }

        std::vector<std::pair<double, call_id_t>> options;
        for (call_id_t call = 1; call <= manipulator.solution.problem.get().n_calls; call++)
        {
            options.push_back({cost_diff[call], call});
        }

        std::vector<call_id_t> removed_calls;
        while (removed_calls.size() < num_elements)
        {
            // call_id_t call = select_best_geom(options, 0.5).second;
            // call_id_t call = select_best(options).second;
            call_id_t call = select_proportionally(options).second;

            options.erase(std::remove_if(options.begin(), options.end(), [call](const std::pair<int, call_id_t>& p) {
                return p.second == call || p.second == -call;
            }), options.end());
            removed_calls.push_back(abs(call));
        }

        for (auto removed_call : removed_calls)
        {
            vehicle_id_t vehicle = manipulator.get_vehicle_for_call(removed_call);
            auto calls = manipulator.calls[vehicle];
            calls.erase(std::remove_if(calls.begin(), calls.end(), [removed_call](call_id_t c) {
                return abs(c) == abs(removed_call);
            }), calls.end());
            manipulator.set_plan(vehicle, calls.begin(), calls.end());
        }

        return removed_calls;
    }
};
