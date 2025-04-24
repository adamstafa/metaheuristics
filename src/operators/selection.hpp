#pragma once

#include <vector>
#include <climits>
#include <random>

#include "../problem/solution.hpp"
#include "rng.hpp"

std::pair<int, call_id_t> select_best_geom(std::vector<std::pair<double, call_id_t>> options, double prob)
{
    // TODO: don't copy stuff, pass options by reference
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

std::pair<int, call_id_t> select_proportionally(std::vector<std::pair<double, call_id_t>> options)
{
    // TODO: don't copy stuff, pass options by reference
    options.erase(std::remove_if(options.begin(), options.end(), [](const std::pair<int, call_id_t>& option) {
        return option.first == INT_MAX;
    }), options.end());

    int min = std::min_element(options.begin(), options.end())->first;
    int max = std::max_element(options.begin(), options.end())->first;

    std::vector<int> probs;
    for (auto opt : options)
    {
        probs.push_back((max - (double) opt.first)/(max - min)); 
    }
    std::discrete_distribution<> d(probs.begin(), probs.end());

    // std::geometric_distribution<> d(prob);
    int index = d(gen) % options.size();
    return options[index];
}
