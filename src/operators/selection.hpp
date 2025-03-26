#pragma once

#include <vector>
#include <climits>
#include <random>

#include "../problem/solution.hpp"
#include "rng.hpp"

std::pair<int, call_id_t> select_best_geom(std::vector<std::pair<double, call_id_t>> options, double prob)
{
    options.erase(std::remove_if(options.begin(), options.end(), [](const std::pair<int, call_id_t>& option) {
        return option.first == INT_MAX;
    }), options.end());
    std::sort(options.begin(), options.end());

    std::geometric_distribution<> d(prob);
    int index = d(gen) % options.size();
    return options[index];
}

std::pair<double, call_id_t> select_best(std::vector<std::pair<double, call_id_t>>& options)
{
    return *std::min_element(options.begin(), options.end());
}
