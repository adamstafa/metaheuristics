#pragma once

#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>
#include <cassert>
#include <iostream>
#include <memory>

#include "problem_reimagined.hpp"


// class Frame
// {
// public:
//     // int arrival_time;
//     int departure_time;
//     int remaining_capacity;
//     call_id_t call;
//     int cost;
//     bool feasible;

//     Frame() {}

//     Frame(int arrival_time, int departure_time, int remaining_capacity, call_id_t call, int cost, bool feasible)
//         : /*arrival_time(arrival_time),*/ departure_time(departure_time), remaining_capacity(remaining_capacity), call(call), cost(cost), feasible(feasible)
//     {
//         // std::cout << "Constructor called\n";
//     }

//     Frame(const Frame& other)
//         : /*arrival_time(other.arrival_time),*/ departure_time(other.departure_time), remaining_capacity(other.remaining_capacity), call(other.call), cost(other.cost), feasible(other.feasible)
//     {
//         // std::cout << "Copy Constructor called\n";
//     }

//     Frame(Frame&& other) noexcept
//         : /*arrival_time(other.arrival_time),*/ departure_time(other.departure_time), remaining_capacity(other.remaining_capacity), call(other.call), cost(other.cost), feasible(other.feasible) 
//     {
//         // std::cout << "Move Constructor called\n";
//     }
// };

class Frame
{
public:
    int departure_time;
    call_id_t call;

    Frame() {}

    Frame(int departure_time, call_id_t call)
        : departure_time(departure_time), call(call)
    {
        // std::cout << "Constructor called\n";
    }

    Frame(const Frame& other)
        : departure_time(other.departure_time), call(other.call)
    {
        // std::cout << "Copy Constructor called\n";
    }

    Frame(Frame&& other) noexcept
        : departure_time(other.departure_time), call(other.call)
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
    int _remaining_capacity;
    int _feasible_size;
    int _cost;
    int _departure_time;
    int _last_call;

    VehicleSolution(vehicle_id_t vehicle, VehicleProblemReimagined &problem)
        : vehicle(vehicle), plan(), problem(problem), _remaining_capacity(problem.vehicle_capacity), _cost(0), _departure_time(problem.starting_time), _last_call(problem.starting_call), _feasible_size(1)
    {
        Frame init_frame{_departure_time, _last_call};
        plan.push_back(init_frame);
    }

    void add_call(call_id_t call_id)
    {
        const auto& call = problem.get().get_call(call_id);
        int travel_time = problem.get().travel_time(_last_call, call_id);
        int travel_cost = problem.get().travel_cost(_last_call, call_id);

        int arrival_time = _departure_time + travel_time;
        _departure_time = std::max(arrival_time, call.window_low) + call.processing_time;
        _remaining_capacity -= call.size;
        _cost += travel_cost + call.processing_cost;
        _last_call = call_id;
        bool _feasible = feasible()
            && _remaining_capacity >= 0
            && arrival_time <= call.window_high
            && call.compatible;
        if (_feasible)
        {
            _feasible_size++;
        }

        Frame new_frame(_departure_time, call_id);
        plan.push_back(new_frame);
    }

    void add_many(std::vector<call_id_t> calls)
    {
        add_many(calls.begin(), calls.end());
        // int offset = plan.size();
        // plan.resize(plan.size() + calls.size());

        // for (int i = 0; i < calls.size(); i++)
        // {
        //     auto call_id = calls[i];
        //     auto& call = problem.get().get_call(call_id);
        //     auto& prev = plan[offset + i - 1];
        //     auto& curr = plan[offset + i];
        //     int travel_time = problem.get().travel_time(prev.call, call_id);
        //     int travel_cost = problem.get().travel_cost(prev.call, call_id);


        //     curr.arrival_time = prev.departure_time + travel_time;
        //     curr.departure_time = std::max(curr.arrival_time, call.window_low) + call.processing_time;
        //     curr.remaining_capacity = prev.remaining_capacity - call.size;
        //     curr.call = call_id;
        //     curr.cost = prev.cost + travel_cost + call.processing_cost;
        //     curr.feasible = prev.feasible
        //         && curr.remaining_capacity >= 0
        //         && curr.arrival_time <= call.window_high
        //         && call.compatible;
        // }
    }

    void add_many(std::vector<call_id_t>::const_iterator begin, std::vector<call_id_t>::const_iterator end)
    {
        int offset = plan.size();
        plan.resize(offset + std::distance(begin, end));

        auto it = begin;
        for (int i = 0; it != end; i++, it++)
        {
            const auto& call = problem.get().get_call(*it);
            int travel_time = problem.get().travel_time(_last_call, call.id);
            int travel_cost = problem.get().travel_cost(_last_call, call.id);

            int arrival_time = _departure_time + travel_time;
            _departure_time = std::max(arrival_time, call.window_low) + call.processing_time;
            _remaining_capacity -= call.size;
            _cost += travel_cost + call.processing_cost;
            _last_call = call.id;
            bool _feasible = (offset + i == _feasible_size)
                && _remaining_capacity >= 0
                && arrival_time <= call.window_high
                && call.compatible;
            if (_feasible)
            {
                _feasible_size++;
            }

            plan[offset + i].call = call.id;
            plan[offset + i].departure_time = _departure_time;
        }
    }

    void remove_many(int n)
    {
        for (int i = 0; i < n; i++)
        {
            remove_call();
        }
    }

    void remove_call()
    {
        auto& call = problem.get().get_call(_last_call);
        auto& prev = plan[plan.size() - 2];
        auto& curr = plan[plan.size() - 1];
        int travel_time = problem.get().travel_time(prev.call, curr.call);
        int travel_cost = problem.get().travel_cost(prev.call, curr.call);

        _departure_time = prev.departure_time;
        _remaining_capacity += call.size;
        _cost -= travel_cost + call.processing_cost;
        _last_call = prev.call;
        if (feasible())
        {
            _feasible_size--;
        }
        plan.pop_back();
    }

    int cost()
    {
        return _cost;
    }

    bool feasible()
    {
        return plan.size() == _feasible_size;
    }

    int num_calls()
    {
        return plan.size() - 1;
    }
};

