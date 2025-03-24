#pragma once

#include <vector>
#include <limits>
#include "problem.hpp"

/*
* Goals:
* - Remove the concept of pickup and delivery - only visit calls    
*   + Calls are now signed: + for the pickup and - for delivery
* - Abstract away from nodes
*   + The route is now a sequence of calls
*   + We now have distances between calls
*/

class VehicleCallReimagined
{
public:
    call_id_t id;
    node_id_t node;
    bool compatible;
    int window_low;
    int window_high;

    int size;
    int processing_time;
    int processing_cost;

    VehicleCallReimagined(call_id_t id, node_id_t node, bool compatible, int window_low, int window_high, int size, int processing_time, int processing_cost)
        : id(id), node(node), compatible(compatible), window_low(window_low), window_high(window_high), size(size), processing_time(processing_time), processing_cost(processing_cost)
    {
    }
};

class VehicleProblemReimagined
{
public:
    vehicle_id_t vehicle_id;
    int vehicle_capacity;
    int starting_call;
    int starting_time;

    int n_calls;
    std::vector<VehicleCallReimagined> calls;
    Matrix travel_times;
    Matrix travel_costs;

    VehicleProblemReimagined(int vehicle_id, int vehicle_capacity, int starting_call, int starting_time, int n_calls, const std::vector<VehicleCallReimagined>& calls, const Matrix& travel_times, const Matrix& travel_costs)
        : vehicle_id(vehicle_id), vehicle_capacity(vehicle_capacity), starting_call(starting_call), starting_time(starting_time), n_calls(n_calls), calls(calls), travel_times(travel_times), travel_costs(travel_costs)
    {
    }

    VehicleProblemReimagined()
        : vehicle_id(0), vehicle_capacity(0), starting_call(0), starting_time(0), n_calls(0), calls(), travel_times(0), travel_costs(0)
    {
    }


    VehicleCallReimagined& get_call(call_id_t call_id)
    {
        return calls[2 * abs(call_id) + (call_id <= 0) - 1];
    }

    int travel_time(call_id_t from, call_id_t to)
    {
        return travel_times(get_call(from).node, get_call(to).node);
    }

    int travel_cost(call_id_t from, call_id_t to)
    {
        return travel_costs(get_call(from).node, get_call(to).node);
    }
};

class ProblemReimagined
{
public:
    int n_vehicles;
    int n_calls;
    std::vector<int> no_transport_costs; // calls 1 ... n_calls
    std::vector<VehicleProblemReimagined> vehicle_problems;

    ProblemReimagined(Problem problem)
        : n_vehicles(problem.n_vehicles), n_calls(problem.n_calls), no_transport_costs(problem.n_calls + 1), vehicle_problems(problem.n_vehicles + 1)
    {
        for (call_id_t i = 1; i <= n_calls; i++)
        {
            no_transport_costs[i] = problem.calls[i].no_transport_cost;
        }

        for (vehicle_id_t v = 1; v <= n_vehicles; v++)
        {
            int capacity = problem.vehicles[v].capacity;
            int starting_call = 0;
            int starting_time = problem.vehicles[v].starting_time;

            std::vector<VehicleCallReimagined> calls;
            calls.push_back(VehicleCallReimagined{0, problem.vehicles[v].home_node, true, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, 0, 0}); // fake call id 0
            for (call_id_t c = 1; c <= n_calls; c++)
            {
                auto &call = problem.calls[c];
                auto &vehicle_call = problem.vehicle_calls[v][c];
                // pickup
                calls.push_back(VehicleCallReimagined{c, call.origin_node, vehicle_call.can_pickup, call.pickup_low, call.pickup_high, call.size, vehicle_call.loading_time, vehicle_call.loading_cost});
                // delivery
                calls.push_back(VehicleCallReimagined{-c, call.destination_node, vehicle_call.can_pickup, call.delivery_low, call.delivery_high, -call.size, vehicle_call.unloading_time, vehicle_call.unloading_cost});
            }

            VehicleProblemReimagined vehicle_problem{v, capacity, starting_call, starting_time, n_calls, calls, problem.travel_times[v], problem.travel_costs[v]};
            vehicle_problems[v] = vehicle_problem;
        }
    }

    int no_transport_cost(call_id_t call)
    {
        if (call < 0)
            return no_transport_costs[abs(call)];
        return 0;
    }
};

