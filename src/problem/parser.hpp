#pragma once

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>

#include "problem.hpp"

Problem parse_problem(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        throw std::runtime_error("Unable to open file");
    }

    std::string skip_line;
    char comma;
    int n_nodes, n_vehicles, n_calls;

    std::getline(file, skip_line);
    file >> n_nodes;
    std::getline(file, skip_line);

    std::getline(file, skip_line);
    file >> n_vehicles;
    std::getline(file, skip_line);

    Problem problem(n_vehicles, n_nodes);

    std::getline(file, skip_line);
    for (int i = 0; i < n_vehicles; i++)
    {
        int vehicle_index, home_node, starting_time, capacity;
        file >> vehicle_index >> comma >> home_node >> comma >> starting_time >> comma >> capacity;
        std::getline(file, skip_line);

        problem.add_vehicle(vehicle_index, home_node, starting_time, capacity);
    }

    std::getline(file, skip_line);
    file >> n_calls;
    std::getline(file, skip_line);

    // uhh, ugly
    std::vector<std::vector<call_id_t>> compatible_calls;
    std::getline(file, skip_line);
    std::string line;
    for (int i = 0; i < n_vehicles; i++)
    {
        std::getline(file, line);
        std::vector<call_id_t> calls;
        std::stringstream ss(line);
        std::string call_str;
        bool first = true;
        while (std::getline(ss, call_str, ',')) {
            if (first)
            {
                first = false;
                continue;
            }
            calls.push_back(std::stoi(call_str));
        }
        compatible_calls.push_back(calls);
    }

    std::getline(file, skip_line);
    for (int i = 0; i < n_calls; i++)
    {
        int call_index, origin_node, destination_node, size, cost_not_transport;
        int lb_pickup, ub_pickup, lb_delivery, ub_delivery;
        file >> call_index >> comma >> origin_node >> comma >> destination_node >> comma >> size >> comma >> cost_not_transport
             >> comma >> lb_pickup >> comma >> ub_pickup >> comma >> lb_delivery >> comma >> ub_delivery;
        std::getline(file, skip_line);

        problem.add_call(call_index, origin_node, destination_node, size, cost_not_transport,
                         lb_pickup, ub_pickup, lb_delivery, ub_delivery);
    }

    std::getline(file, skip_line);
    
    for (int i = 0; i < n_nodes * n_nodes * n_vehicles; i++ )
    {
        int vehicle, origin_node, destination_node, travel_time, travel_cost;
        file >> vehicle >> comma >> origin_node >> comma >> destination_node >> comma >> travel_time >> comma >> travel_cost;
        std::getline(file, skip_line);

        problem.add_connection(vehicle, origin_node, destination_node, travel_time, travel_cost);
    }

    std::getline(file, skip_line);
    for (int i = 0; i < n_calls * n_vehicles; i++)
    {
        int vehicle, call, origin_node_time, origin_node_cost, destination_node_time, destination_node_cost;
        file >> vehicle >> comma >> call >> comma >> origin_node_time >> comma >> origin_node_cost >> comma >> destination_node_time >> comma >> destination_node_cost;
        std::getline(file, skip_line);

        auto &cc = compatible_calls[vehicle - 1];
        bool can_pickup = std::find(cc.begin(), cc.end(), call) != cc.end();

        problem.add_call_params(vehicle, call, can_pickup, origin_node_time, origin_node_cost, destination_node_time, destination_node_cost);
    }

    return std::move(problem);
}
