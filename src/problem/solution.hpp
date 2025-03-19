#pragma once

#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>
#include <cassert>

#include "problem_reimagined.hpp"


class Frame
{
public:
    int arrival_time;
    int departure_time;
    int remaining_capacity;
    call_id_t call;
    int cost;
    bool feasible;

    Frame(int arrival_time, int departure_time, int remaining_capacity, call_id_t call, int cost, bool feasible)
        : arrival_time(arrival_time), departure_time(departure_time), remaining_capacity(remaining_capacity), call(call), cost(cost), feasible(feasible)
    {
    }

    Frame(const Frame &other)
        : arrival_time(other.arrival_time), departure_time(other.departure_time), remaining_capacity(other.remaining_capacity), call(other.call), cost(other.cost), feasible(other.feasible)
    {
    }
};

class VehicleSolution
{
public:
    vehicle_id_t vehicle;
    std::vector<Frame> plan;
    std::reference_wrapper<VehicleProblemReimagined> problem;

    VehicleSolution(vehicle_id_t vehicle, VehicleProblemReimagined &problem)
        : vehicle(vehicle), plan(), problem(problem)
    {
        Frame init_frame{0, problem.starting_time, problem.vehicle_capacity, problem.starting_call, 0, true};
        plan.push_back(init_frame);
    }

    void add_call(call_id_t call_id)
    {
        Frame &last = plan.back();
        Frame new_frame(last);
        auto &call = problem.get().get_call(call_id);

        int travel_time = problem.get().travel_time(last.call, call_id);
        int travel_cost = problem.get().travel_cost(last.call, call_id);

        new_frame.arrival_time = last.departure_time + travel_time;
        new_frame.departure_time = std::max(new_frame.arrival_time, call.window_low) + call.processing_time;
        new_frame.remaining_capacity -= call.size;
        new_frame.call = call_id;
        new_frame.cost += travel_cost + call.processing_cost;
        new_frame.feasible = last.feasible
            && new_frame.remaining_capacity >= 0
            // && new_frame.arrival_time >= call.window_low
            && new_frame.arrival_time <= call.window_high
            && call.compatible;
        // TODO: consider storing debug information such as within_window or reason for incompatibility

        plan.push_back(new_frame);
    }

    void remove_call()
    {
        plan.pop_back();
    }

    int cost()
    {
        return plan.back().cost;
    }

    bool feasible()
    {
        return plan.back().feasible;
    }

    int num_calls()
    {
        return plan.size() - 1;
    }
};

class Solution
{
public:
    std::vector<std::shared_ptr<VehicleSolution>> vehicle_solutions;
    std::reference_wrapper<ProblemReimagined>problem;
    int plan_cost;
    int no_transport_cost;
    int feasible_count;
    std::vector<int> picked_up;
    std::vector<int> delivered;

    Solution(ProblemReimagined &problem)
        : vehicle_solutions(), problem(problem), plan_cost(0), feasible_count(problem.n_vehicles), picked_up(problem.n_calls + 1), delivered(problem.n_calls + 1)
    {
        for (int v = 1; v <= problem.n_vehicles; v++)
        {
            vehicle_solutions.push_back(std::make_shared<VehicleSolution>(v, problem.vehicle_problems[v]));
        }
        no_transport_cost = std::accumulate(problem.no_transport_cost.begin(), problem.no_transport_cost.end(), 0);
    }

    void push_call(vehicle_id_t vehicle, call_id_t call)
    {
        assert(vehicle >= 1 && vehicle <= problem.get().n_vehicles);
        assert(call >= 1 && call <= problem.get().n_calls);
        auto &sol = *vehicle_solutions[vehicle - 1];
        
        int call_to_insert;
        if (picked_up[call] == 0)
        {
            assert(delivered[call] == 0);
            call_to_insert = call;
            picked_up[call] = vehicle;
        }
        else
        {
            assert(picked_up[call] == vehicle);
            assert(delivered[call] == 0);
            call_to_insert = -call;
            delivered[call] = vehicle;
            no_transport_cost -= problem.get().no_transport_cost[call];
        }

        int prev_cost = sol.cost();
        bool prev_feasible = sol.feasible();

        sol.add_call(call_to_insert);
        plan_cost = plan_cost - prev_cost + sol.cost();
        feasible_count = feasible_count - prev_feasible + sol.feasible();
    };

    void pop_call(vehicle_id_t vehicle)
    {
        assert(vehicle >= 1 && vehicle <= problem.get().n_vehicles); // TODO: do I need to call get?
        auto &sol = *vehicle_solutions[vehicle - 1];

        auto& last = sol.plan.back();
        int call_id = abs(last.call);

        if (last.call > 0)
        {
            assert(picked_up[call_id] == vehicle);
            assert(delivered[call_id] == 0);
            picked_up[call_id] = 0;
        }
        else
        {
            assert(delivered[call_id] == vehicle);
            assert(picked_up[call_id] == vehicle);
            delivered[call_id] = 0;
            no_transport_cost += problem.get().no_transport_cost[call_id];
        }

        int prev_cost = sol.cost();
        bool prev_feasible = sol.feasible();
        
        sol.remove_call();

        plan_cost = plan_cost - prev_cost + sol.cost();
        feasible_count = feasible_count - prev_feasible + sol.feasible();
    };

    void clone_vehicle_solution(vehicle_id_t vehicle)
    {
        auto new_vehicle_solution = std::make_shared<VehicleSolution>(*vehicle_solutions[vehicle - 1]);
        vehicle_solutions[vehicle - 1] = new_vehicle_solution;
    }

    VehicleSolution& vehicle_solution(vehicle_id_t vehicle)
    {
        return *vehicle_solutions[vehicle-1];
    }

    int cost()
    {
        return plan_cost + no_transport_cost;
    }

    bool valid()
    {
        // TODO: check that every call is either not picked up or delivered

        return true;
    }

    bool feasible()
    {
        return feasible_count == problem.get().n_vehicles;
    }

    void print() const
    {
        for (const auto &vehicle_solution : vehicle_solutions)
        {
            for (const auto &frame : vehicle_solution->plan)
            {
                std::cout << abs(frame.call) << " ";
            }
            std::cout << std::endl;
        }
    }
};
