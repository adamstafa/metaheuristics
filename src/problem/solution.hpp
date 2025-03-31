#pragma once

#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>
#include <cassert>
#include <iostream>
#include <memory>

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

    Frame() {}

    Frame(int arrival_time, int departure_time, int remaining_capacity, call_id_t call, int cost, bool feasible)
        : arrival_time(arrival_time), departure_time(departure_time), remaining_capacity(remaining_capacity), call(call), cost(cost), feasible(feasible)
    {
        // std::cout << "Constructor called\n";
    }

    Frame(const Frame& other)
        : arrival_time(other.arrival_time), departure_time(other.departure_time), remaining_capacity(other.remaining_capacity), call(other.call), cost(other.cost), feasible(other.feasible)
    {
        // std::cout << "Copy Constructor called\n";
    }

    Frame(Frame&& other) noexcept
        : arrival_time(other.arrival_time), departure_time(other.departure_time), remaining_capacity(other.remaining_capacity), call(other.call), cost(other.cost), feasible(other.feasible) 
    {
        // std::cout << "Move Constructor called\n";
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
        const auto& last = plan.back();
        const auto& call = problem.get().get_call(call_id);
        int travel_time = problem.get().travel_time(last.call, call_id);
        int travel_cost = problem.get().travel_cost(last.call, call_id);

        int arrival_time = last.departure_time + travel_time;
        int departure_time = std::max(arrival_time, call.window_low) + call.processing_time;
        int remaining_capacity = last.remaining_capacity - call.size;
        int cost = last.cost + travel_cost + call.processing_cost;
        bool feasible = last.feasible
            && remaining_capacity >= 0
            && arrival_time <= call.window_high
            && call.compatible;

        // TODO: why is this faster than emplace_back???
        Frame new_frame(arrival_time, departure_time, remaining_capacity, call_id, cost, feasible);
        plan.push_back(new_frame);
        // plan.emplace_back(arrival_time, departure_time, remaining_capacity, call_id, cost, feasible);
    }

    void add_many(std::vector<call_id_t>::const_iterator begin, std::vector<call_id_t>::const_iterator end)
    {
        int offset = plan.size();
        plan.resize(offset + std::distance(begin, end));

        for (int i = 0; begin != end; i++, begin++)
        {
            auto call_id = *begin;
            auto& call = problem.get().get_call(call_id);
            auto& prev = plan[offset + i - 1];
            auto& curr = plan[offset + i];
            int travel_time = problem.get().travel_time(prev.call, call_id);
            int travel_cost = problem.get().travel_cost(prev.call, call_id);


            curr.arrival_time = prev.departure_time + travel_time;
            curr.departure_time = std::max(curr.arrival_time, call.window_low) + call.processing_time;
            curr.remaining_capacity = prev.remaining_capacity - call.size;
            curr.call = call_id;
            curr.cost = prev.cost + travel_cost + call.processing_cost;
            curr.feasible = prev.feasible
                && curr.remaining_capacity >= 0
                && curr.arrival_time <= call.window_high
                && call.compatible;
        }
    }

    void remove_many(int n)
    {
        plan.resize(plan.size() - n);
    }

    void remove_call()
    {
        plan.pop_back();
    }

    void reserve(int n)
    {
        plan.reserve(n + 1);
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

    Solution(ProblemReimagined &problem)
        : vehicle_solutions(), problem(problem), plan_cost(0), feasible_count(problem.n_vehicles)
    {
        for (int v = 1; v <= problem.n_vehicles; v++)
        {
            vehicle_solutions.push_back(std::make_shared<VehicleSolution>(v, problem.vehicle_problems[v]));
        }
        no_transport_cost = std::accumulate(problem.no_transport_costs.begin(), problem.no_transport_costs.end(), 0);
    }

    void push_call(vehicle_id_t vehicle, call_id_t call)
    {
        assert(vehicle >= 1 && vehicle <= problem.get().n_vehicles);
        assert(abs(call) >= 1 && abs(call) <= problem.get().n_calls);
        auto &sol = *vehicle_solutions[vehicle - 1];
        
        no_transport_cost -= problem.get().no_transport_cost(call);

        int prev_cost = sol.cost();
        bool prev_feasible = sol.feasible();

        sol.add_call(call);
        plan_cost = plan_cost - prev_cost + sol.cost();
        feasible_count = feasible_count - prev_feasible + sol.feasible();
    }

    void pop_call(vehicle_id_t vehicle)
    {
        assert(vehicle >= 1 && vehicle <= problem.get().n_vehicles); // TODO: do I need to call get?
        auto &sol = *vehicle_solutions[vehicle - 1];

        int call = sol.plan.back().call;
        no_transport_cost += problem.get().no_transport_cost(call);

        int prev_cost = sol.cost();
        bool prev_feasible = sol.feasible();
        
        sol.remove_call();

        plan_cost = plan_cost - prev_cost + sol.cost();
        feasible_count = feasible_count - prev_feasible + sol.feasible();
    }

    void set_vehicle_plan(vehicle_id_t vehicle, std::vector<call_id_t>::const_iterator begin, std::vector<call_id_t>::const_iterator end)
    {
        auto &sol = *vehicle_solutions[vehicle - 1];
        int prev_cost = sol.cost();
        bool prev_feasible = sol.feasible();
        
        int same = 0;
        int count = std::distance(begin, end);
        auto it = begin;
        while (same < count  && same < sol.num_calls() && abs(sol.plan[same + 1].call) == *it)
        {
            same++;
            it++;
        }

        for (int i = same; i < sol.num_calls(); i++)
        {
            auto call = sol.plan[i + 1].call;
            no_transport_cost += problem.get().no_transport_cost(call);
        }

        for (auto it = begin + same; it != end; it++)
        {
            no_transport_cost -= problem.get().no_transport_cost(*it);
        }
        sol.remove_many(sol.num_calls() - same);
        sol.add_many(begin + same, end);
        
        plan_cost = plan_cost - prev_cost + sol.cost();
        feasible_count = feasible_count - prev_feasible + sol.feasible();
    }

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

    std::string python_string()
    {
        std::vector<bool> unhandled_calls(problem.get().n_calls + 1, true);
        std::vector<int> joined_calls;
        for (vehicle_id_t v = 1; v <= problem.get().n_vehicles; v++)
        {
            auto& vs = vehicle_solution(v);
            for (int i = 0; i < vs.num_calls(); i++)
            {
                auto call = vs.plan[i + 1].call;
                if (call != 0)
                {
                    unhandled_calls[abs(call)] = false;
                    joined_calls.push_back(abs(call));
                }
            }
            joined_calls.push_back(0);
        }
        for (call_id_t c = 1; c <= problem.get().n_calls; c++)
        {
            if (unhandled_calls[c])
            {
                joined_calls.push_back(c);
                joined_calls.push_back(c);
            }
        }

        std::string result = "[";
        for (size_t i = 0; i < joined_calls.size(); ++i)
        {
            result += std::to_string(joined_calls[i]);
            if (i != joined_calls.size() - 1)
            {
                result += ", ";
            }
        }
        result += "]";
        return result;
    }
};
