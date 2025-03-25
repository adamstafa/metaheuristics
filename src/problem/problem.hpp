#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <map>

typedef int vehicle_id_t;
typedef int call_id_t;
typedef int node_id_t;

class Vehicle
{
public:
    vehicle_id_t id;
    node_id_t home_node;
    int starting_time;
    int capacity;

    Vehicle() = default;

    Vehicle(vehicle_id_t id, int home_node, int starting_time, int capacity)
        : id(id), home_node(home_node), starting_time(starting_time), capacity(capacity)
    {
    }
};

class Call
{
public:
    call_id_t id;
    node_id_t origin_node;
    node_id_t destination_node;
    int size;
    int no_transport_cost;
    int pickup_low;
    int pickup_high;
    int delivery_low;
    int delivery_high;

    Call() = default;

    Call(call_id_t id, int origin_node, int destination_node, int size, int no_transport_cost, int pickup_low, int pickup_high, int delivery_low, int delivery_high)
        : id(id), origin_node(origin_node), destination_node(destination_node), size(size), no_transport_cost(no_transport_cost), pickup_low(pickup_low), pickup_high(pickup_high), delivery_low(delivery_low), delivery_high(delivery_high)
    {
    }
};

class Matrix
{
public:
    int n_nodes;
    std::vector<int> data;

    Matrix(int n_nodes)
        : n_nodes(n_nodes), data(n_nodes * n_nodes)
    {
    }

    int operator()(int i, int j) const
    {
        return data[(i - 1) * n_nodes + (j - 1)];
    }

    int &operator()(int i, int j)
    {
        return data[(i - 1) * n_nodes + (j - 1)];
    }
};

class VehicleCall
{
public:
    bool can_pickup;
    int loading_time;   // origin node time
    int unloading_time; // destination node time
    int loading_cost;   // origin node cost
    int unloading_cost; // destination node cost

    VehicleCall() = default;

    VehicleCall(bool can_pickup, int loading_time, int unloading_time, int loading_cost, int unloading_cost)
        : can_pickup(can_pickup), loading_time(loading_time), unloading_time(unloading_time), loading_cost(loading_cost), unloading_cost(unloading_cost) {};
};

class Problem
{
public:
    int n_vehicles;
    int n_calls;
    int n_nodes;

    std::vector<Vehicle> vehicles;
    std::vector<Call> calls;

    std::vector<Matrix> travel_times; // vehicle_id
    std::vector<Matrix> travel_costs; // vehicle_id

    std::map<vehicle_id_t, std::map<call_id_t, VehicleCall>> vehicle_calls;

    Problem(int n_vehicles, int n_nodes)
        : n_vehicles(n_vehicles), n_calls(0), n_nodes(n_nodes), vehicles(), calls(), travel_times(), travel_costs(), vehicle_calls()
    {
        vehicles.resize(n_vehicles + 1);
        travel_times.resize(n_vehicles + 1, n_nodes);
        travel_costs.resize(n_vehicles + 1, n_nodes);
        calls.resize(1); // calls will be populated after constructing the problem
    }

    void add_vehicle(vehicle_id_t id, int home_node, int starting_time, int capacity)
    {
        vehicles[id] = Vehicle(id, home_node, starting_time, capacity);
    }

    void add_call(call_id_t id, int origin_node, int destination_node, int size, int no_transport_cost, int pickup_low, int pickup_high, int delivery_low, int delivery_high)
    {
        calls.push_back(Call(id, origin_node, destination_node, size, no_transport_cost, pickup_low, pickup_high, delivery_low, delivery_high));
        n_calls++;
    }

    void add_connection(vehicle_id_t vehicle, node_id_t origin_node, node_id_t destination_node, int travel_time, int travel_cost)
    {
        travel_times[vehicle](origin_node, destination_node) = travel_time;
        travel_costs[vehicle](origin_node, destination_node) = travel_cost;
    }

    void add_call_params(vehicle_id_t vehicle, call_id_t call, bool can_pickup, int origin_node_time, int origin_node_cost, int destination_node_time, int destination_node_cost)
    {
        vehicle_calls[vehicle][call] = VehicleCall(can_pickup, origin_node_time, destination_node_time, origin_node_cost, destination_node_cost);
    }
};
