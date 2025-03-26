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
    int num_elements;

    BaseRemover(SolutionManipulator& manipulator, int num_elements)
        : manipulator(manipulator), num_elements(num_elements) {}

    virtual std::vector<call_id_t> remove() = 0;
};

class RandomRemover : BaseRemover
{
public:
    RandomRemover(SolutionManipulator& manipulator, int num_elements) : BaseRemover(manipulator, num_elements) {}

    std::vector<call_id_t> remove() override
    {
        std::vector<call_id_t> removed_calls;

        while (removed_calls.size() < num_elements)
        {
            int vehicle = (rand() % 100 <= 20) ? 0 : ((rand() % manipulator.solution.problem.get().n_vehicles) + 1);
            auto calls = manipulator.calls[vehicle];
            if (calls.size() == 0)
                continue;
            
            int index = rand() % calls.size();
            call_id_t removed_call = calls[index];
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
    SimilarVehiclesRemover(SolutionManipulator& manipulator, int num_elements) : BaseRemover(manipulator, num_elements) {}

    std::vector<call_id_t> remove()
    {
        std::vector<call_id_t> removed_calls;

        call_id_t first_call = rand() % manipulator.solution.problem.get().n_calls + 1;
        std::vector<std::pair<double, call_id_t>> distances;
        for (call_id_t call = 1; call <= manipulator.solution.problem.get().n_calls; call++)
        {
            distances.push_back({similarity_score(first_call, call, manipulator), call});
        }

        while (removed_calls.size() < num_elements)
        {
            call_id_t call = select_best_geom(distances, 0.2).second;
            distances.erase(std::remove_if(distances.begin(), distances.end(), [call](const std::pair<int, call_id_t>& p) {
                return abs(p.second) == abs(call);
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
    
    double similarity_score(call_id_t call_1_id, call_id_t call_2_id, SolutionManipulator& manipulator)
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

        return (cargo_similarity + distance_similarity + time_similarity + compatibility_similarity) / 4.0;
    }
};

class FullVehiclesRemover : BaseRemover
{
public:
    FullVehiclesRemover(SolutionManipulator& manipulator, int num_elements) : BaseRemover(manipulator, num_elements) {}

    std::vector<call_id_t> remove()
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
