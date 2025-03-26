#pragma once

#include <memory>
#include "../problem/solution_manipulator.hpp"
#include <random>
#include <climits>

// TODO: split up the file


// random number generator
std::random_device rd;
std::mt19937 gen(rd());

class BaseOperator
{
public:
    SolutionManipulator& manipulator;

    BaseOperator(SolutionManipulator& manipulator) : manipulator(manipulator) { srand(time(nullptr)); } // TODO: dont initialize random like this

    virtual void apply() = 0;
};

class Sequence : public BaseOperator
{
public:
    std::vector<std::unique_ptr<BaseOperator>> operators;

    Sequence(SolutionManipulator& manipulator, std::vector<std::unique_ptr<BaseOperator>> ops) 
        : BaseOperator(manipulator), operators(std::move(ops)) {}

    void apply() override
    {
        for (auto& op : operators)
        {
            op->apply();
        }
    }
};


std::pair<int, std::vector<call_id_t>> calculate_insertion_cost(call_id_t call_id, vehicle_id_t vehicle, SolutionManipulator& manipulator)
{
    if (vehicle == 0)
    {
        auto calls = manipulator.calls[0];
        calls.push_back(call_id);
        calls.push_back(-call_id);
        return {manipulator.solution.problem.get().no_transport_costs[call_id], std::move(calls)};
    }

    auto &og_vs = manipulator.solution.vehicle_solution(vehicle);
    if (!og_vs.problem.get().get_call(call_id).compatible)
    {
        return {INT_MAX, {}};
    }

    std::pair<int, std::vector<call_id_t>> best {INT_MAX, {}};
    VehicleSolution vs{vehicle, manipulator.solution.problem.get().vehicle_problems[vehicle]};
    auto& pickup = vs.problem.get().get_call(call_id);
    auto& delivery = vs.problem.get().get_call(-call_id);
    std::vector<call_id_t> calls = manipulator.calls[vehicle];

    std::vector<int> latest_arrival(calls.size());
    int last = INT_MAX;
    for (int i = calls.size() - 1; i >= 0; i--)
    {
        last = std::min(last, vs.problem.get().get_call(calls[i]).window_high);
        latest_arrival[i] = last;
    }

    for (int i = 0; i <= calls.size(); i++) // i = number of calls before the first insertion place
    {
        if (i > 0)
        {
            vs.remove_many(vs.num_calls() - (i - 1));
            vs.add_call(calls[i - 1]);
        }


        if (vs.plan.back().departure_time > std::min(pickup.window_high, delivery.window_high))
        {
            break;
        }

        vs.add_call(call_id);
    
        if (i < calls.size() && vs.plan.back().departure_time > latest_arrival[i])
        {
            continue;
        }

        for (int j = 0; j <= calls.size() - i; j++) // j = number of calls between the insertion places
        {
            if (j > 0)
            {
                vs.remove_many(vs.num_calls() - (i + j));
                vs.add_call(calls[i + j - 1]);
            }

            if (vs.plan.back().departure_time > delivery.window_high)
            {
                break;
            }

            vs.add_call(-call_id);

            if ((i + j) < calls.size() && vs.plan.back().departure_time > latest_arrival[i + j])
            {
                continue;
            }

            vs.add_many(calls.begin() + i + j, calls.end());

            if (vs.feasible() && vs.cost() < best.first)
            {
                // save found solution
                std::vector<call_id_t> new_calls;
                for (int i = 1; i <= vs.num_calls(); i++)
                {
                    new_calls.push_back(vs.plan[i].call);
                }
                best = {vs.cost(), std::move(new_calls)};
            }
        }
    }

    // subtract original cost of the vehicle - we care how much more expensive the solution becomes
    best.first -= manipulator.solution.vehicle_solution(vehicle).cost();
    return std::move(best);
}

std::vector<call_id_t> remove_calls_randomly(SolutionManipulator& manipulator, int num_elements)
{
    std::vector<call_id_t> removed_calls;

    while (removed_calls.size() < num_elements)
    {
        int vehicle = (rand() % 100 <= 2) ? 0 : ((rand() % manipulator.solution.problem.get().n_vehicles) + 1);
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

std::pair<double, call_id_t> select_best(std::vector<std::pair<double, call_id_t>> options)
{
    return *std::min_element(options.begin(), options.end());
}

class RegretInserter
{
public:
    SolutionManipulator& manipulator;
    std::vector<std::vector<int>> costs; // [call][vehicle]
    std::vector<std::vector<std::vector<call_id_t>>> plans;
    ProblemReimagined& problem;

    RegretInserter(SolutionManipulator& manipulator) : manipulator(manipulator), costs(), plans(), problem(manipulator.solution.problem.get())
    {
        costs.push_back({});
        plans.push_back({});
        for (int c = 1; c <= problem.n_calls; c++)
        {
            costs.push_back({});
            plans.push_back({});
            for (int v = 0; v <= problem.n_vehicles; v++)
            {
                costs[c].push_back(INT_MAX);
                plans[c].push_back({});
            }
        }
    };

    void insert(std::vector<call_id_t> calls)
    {
        for (auto call : calls)
        {
            for (vehicle_id_t v = 0; v <= problem.n_vehicles; v++)
            {
                auto x = calculate_insertion_cost(call, v, manipulator); // TODO: rename
                costs[call][v] = x.first;
                plans[call][v] = std::move(x.second);
            }
        }

        while (calls.size() > 0)
        {
            insert_one(calls);
        }
    }

    void insert_one(std::vector<call_id_t>& calls)
    {
        std::vector<int> regret;

        for (auto call : calls)
        {
            std::vector<int> call_costs = costs[call];
            std::sort(call_costs.begin(), call_costs.end());
            regret.push_back(call_costs[1] - call_costs[0]);
        }

        std::vector<std::pair<double, call_id_t>> options;
        for (int i = 0; i < calls.size(); i++)
        {
            options.push_back({-regret[i], calls[i]});
        }

        call_id_t best_call = select_best_geom(options, 0.5).second;

        int best_cost = INT_MAX;
        vehicle_id_t best_vehicle;

        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            if (costs[best_call][v] < best_cost)
            {
                best_vehicle = v;
                best_cost = costs[best_call][v];
            }
        }

        auto& best_plan = plans[best_call][best_vehicle];

        manipulator.set_plan(best_vehicle, best_plan.begin(), best_plan.end());

        calls.erase(std::remove_if(calls.begin(), calls.end(), [best_call](call_id_t c) {
            return abs(c) == abs(best_call);
        }), calls.end());

        for (auto call : calls)
        {
            auto x = calculate_insertion_cost(call, best_vehicle, manipulator); // TODO: rename
            costs[call][best_vehicle] = x.first;
            plans[call][best_vehicle] = std::move(x.second);
        }
    }

};


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


std::vector<call_id_t> remove_similar_vehicles(SolutionManipulator& manipulator, int num_elements)
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

std::vector<call_id_t> remove_full_vehicles(SolutionManipulator& manipulator, int num_elements)
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

class ReinsertRandomRegret : public BaseOperator
{
public:
    int num_elements;
    RegretInserter inserter;

    ReinsertRandomRegret(SolutionManipulator& manipulator, int num_elements) : BaseOperator(manipulator),
        num_elements(std::min(num_elements, manipulator.solution.problem.get().n_calls)), inserter(manipulator) {}

    void apply() override
    {
        auto removed_calls = remove_calls_randomly(manipulator, num_elements);
        inserter.insert(std::move(removed_calls));
    }
};

class ReinsertSimilarRegret : public BaseOperator
{
public:
    int num_elements;
    RegretInserter inserter;

    ReinsertSimilarRegret(SolutionManipulator& manipulator, int num_elements) : BaseOperator(manipulator),
        num_elements(std::min(num_elements, manipulator.solution.problem.get().n_calls)), inserter(manipulator) {}

    void apply() override
    {
        auto removed_calls = remove_similar_vehicles(manipulator, num_elements);
        inserter.insert(std::move(removed_calls));
    }
};

class ReinsertFullRegret : public BaseOperator
{
public:
    int num_elements;
    RegretInserter inserter;

    ReinsertFullRegret(SolutionManipulator& manipulator, int num_elements) : BaseOperator(manipulator),
        num_elements(std::min(num_elements, manipulator.solution.problem.get().n_calls)), inserter(manipulator) {}

    void apply() override
    {
        auto removed_calls = remove_full_vehicles(manipulator, num_elements);
        inserter.insert(std::move(removed_calls));
    }
};

class RandomChoice : public BaseOperator
{
public:
    std::vector<std::unique_ptr<BaseOperator>> operators;

    RandomChoice(SolutionManipulator& manipulator, std::vector<std::unique_ptr<BaseOperator>> ops) 
        : BaseOperator(manipulator), operators(std::move(ops)) {}

    void apply() override
    {
        std::uniform_int_distribution<> dis(0, operators.size() - 1);
        int random_index = dis(gen);
        operators[random_index]->apply();
    }
};


class WeightedRandomChoice : public BaseOperator
{
public:
    std::vector<std::unique_ptr<BaseOperator>> operators;
    std::vector<double> weights;

    WeightedRandomChoice(SolutionManipulator& manipulator, std::vector<std::unique_ptr<BaseOperator>> ops, std::vector<double> weights) 
        : BaseOperator(manipulator), operators(std::move(ops)), weights(weights)
    {
        assert(operators.size() == weights.size());
    }

    void apply() override
    {
        std::discrete_distribution<> dis(weights.begin(), weights.end());
        int random_index = dis(gen);
        operators[random_index]->apply();
    }
};
