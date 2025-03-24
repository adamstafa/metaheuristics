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

    BaseOperator(SolutionManipulator& manipulator) : manipulator(manipulator) { srand(time(nullptr));} // TODO: dont initialize random like this

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

void compute_best_insertion_place(VehicleSolution& vs, std::vector<call_id_t>& remaining, std::pair<int, std::vector<call_id_t>>& current, std::pair<int, std::vector<call_id_t>>& best)
{
    auto& problem = vs.problem.get();
    // skip if infeasible
    if (!vs.feasible())
    {
        return ;
    }
    // skip if solution can't be finished
    for (auto rem : remaining)
    {
        if (problem.get_call(rem).window_high < vs.plan.back().departure_time)
        {
            return;
        }
    }

    if (remaining.empty() && vs.feasible() && vs.cost() < best.first)
    {
        best = current;
    }

    for (auto c : remaining)
    {
        // not picked up yet
        if (c < 0 && std::find(remaining.begin(), remaining.end(), -c) != remaining.end())
        {
            continue;
        }

        current.second.push_back(c);
        
        std::vector<call_id_t> new_remaining;
        for (auto rem : remaining)
        {
            if (rem != c)
            {
            new_remaining.push_back(rem);
            }
        }

        vs.add_call(c);
        compute_best_insertion_place(vs, new_remaining, current, best);
        vs.remove_call();

        current.second.pop_back();
    }
}

std::pair<int, std::vector<call_id_t>> compute_best_permutation(VehicleSolution& vehicle_solution_ref)
{
    VehicleSolution vs{vehicle_solution_ref.vehicle, vehicle_solution_ref.problem};
    std::vector<call_id_t> calls;
    for (int i = 0; i < vehicle_solution_ref.num_calls(); i++)
    {
        calls.push_back(vehicle_solution_ref.plan[i+1].call);
    }
    
    std::pair<int, std::vector<call_id_t>> current, best;
    best.first = INT_MAX;

    compute_best_insertion_place(vs, calls, current, best);

    return best;
}

class BestPermutationOperator : public BaseOperator
{
public:
    BestPermutationOperator(SolutionManipulator& manipulator) : BaseOperator(manipulator) {}

    void apply() override
    {
        for (auto& vs : manipulator.solution.vehicle_solutions)
        {
            auto best_permutation = compute_best_permutation(*vs).second;
            manipulator.set_plan(vs->vehicle, best_permutation);
        }
    }
};


// goal: write greedy reinsert algorithm
// input: set of several calls
// for each call...
//   go through all compatible vehicles
//   try to insert the calls on all possible places (there will be only a few feasible places, do fast checks)
//   mark the best places for each call
//
// then greedily insert the calls one by one in randomized order
// 

std::pair<int, std::vector<call_id_t>> calculate_insertion_cost_all_permutations(call_id_t call, vehicle_id_t vehicle, SolutionManipulator manipulator)
{
    if (vehicle == 0)
    {
        auto calls = manipulator.calls[0];
        calls.push_back(call);
        calls.push_back(call);
        return {manipulator.solution.problem.get().no_transport_costs[call], calls};
    }

    VehicleSolution vs{vehicle, manipulator.solution.problem.get().vehicle_problems[vehicle]};

    std::vector<call_id_t> calls;
    for (int i = 0; i < manipulator.solution.vehicle_solution(vehicle).num_calls(); i++)
    {
        calls.push_back(manipulator.solution.vehicle_solution(vehicle).plan[i+1].call);
    }

    // these are signed calls on level of VehicleSolution
    calls.push_back(call);
    calls.push_back(-call);
    
    std::pair<int, std::vector<call_id_t>> current, best;
    best.first = INT_MAX;

    compute_best_insertion_place(vs, calls, current, best);
    int cost = best.first - manipulator.solution.vehicle_solution(vehicle).cost();

    return {cost, best.second};
}


std::pair<int, std::vector<call_id_t>> calculate_insertion_cost(call_id_t call_id, vehicle_id_t vehicle, SolutionManipulator manipulator)
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
    std::vector<call_id_t> calls;
    for (int i = 0; i < manipulator.solution.vehicle_solution(vehicle).num_calls(); i++)
    {
        calls.push_back(manipulator.solution.vehicle_solution(vehicle).plan[i+1].call); // now it's just manipulator.calls[vehicle]
    }

    std::vector<int> latest_arrival(calls.size());
    int last = INT_MAX;
    for (int i = calls.size() - 1; i >= 0; i--)
    {
        last = std::min(last, vs.problem.get().get_call(calls[i]).window_high);
        latest_arrival[i] = last;
    }

    for (int i = 0; i <= calls.size(); i++) // i = number of calls before the first insertion place
    {
        vs.remove_many(vs.num_calls());
        vs.add_many(calls.begin(), calls.begin() + i);

        if (vs.plan.back().departure_time > pickup.window_high || vs.plan.back().departure_time > delivery.window_high)
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
            vs.remove_many(vs.num_calls());
            vs.add_many(calls.begin(), calls.begin() + i);
            vs.add_call(call_id);
            vs.add_many(calls.begin() + i, calls.begin() + i + j);

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
    return best;
}

std::vector<call_id_t> remove_calls(SolutionManipulator& manipulator, int num_elements)
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
        manipulator.set_plan(vehicle, calls);
    }

    return removed_calls;
}

void greedy_insert(std::vector<call_id_t> calls, SolutionManipulator &manipulator)
{
    auto& problem = manipulator.solution.problem.get();
 
    std::vector<std::vector<std::pair<int, std::vector<call_id_t>>>> costs; // costs[c][v] is the cost of inserting call c into vehicle v
    costs.push_back({});
    for (int c = 1; c <= problem.n_calls; c++)
    {
        costs.push_back({});
        
        if (std::find(calls.begin(), calls.end(), c) == calls.end())
        {
            continue;
        }
        // TODO: space wasted in the array

        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            costs[c].push_back(calculate_insertion_cost_all_permutations(c, v, manipulator));
        }
    }

    std::shuffle(calls.begin(), calls.end(), gen);

    while (calls.size() > 0)
    {
        call_id_t call = calls.back();
        calls.pop_back();

        std::pair<int, std::vector<call_id_t>> best_plan;
        best_plan.first = INT_MAX;
        vehicle_id_t best_vehicle = -1;

        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            if (costs[call][v] < best_plan)
            {
                best_plan = costs[call][v];
                best_vehicle = v;
            }
        }

        manipulator.set_plan(best_vehicle, best_plan.second);

        for (auto c : calls)
        {
            costs[c][best_vehicle] = calculate_insertion_cost_all_permutations(c, best_vehicle, manipulator);
        }
    }
}

std::pair<int, call_id_t> select_best_geom(std::vector<std::pair<double, call_id_t>> options, double prob)
{
    std::sort(options.begin(), options.end());
    options.erase(std::remove_if(options.begin(), options.end(), [](const std::pair<int, call_id_t>& option) {
        return option.first == INT_MAX;
    }), options.end());

    std::geometric_distribution<> d(prob);
    int index = d(gen) % options.size();
    return options[index];
}

std::pair<int, call_id_t> select_best(std::vector<std::pair<int, call_id_t>> options)
{
    return *std::min_element(options.begin(), options.end());
}

void regret_insert(std::vector<call_id_t> calls, SolutionManipulator &manipulator)
{
    auto& problem = manipulator.solution.problem.get();
 
    std::vector<std::vector<std::pair<int, std::vector<call_id_t>>>> costs; // costs[c][v] is the cost of inserting call c into vehicle v
    costs.push_back({});
    for (int c = 1; c <= problem.n_calls; c++)
    {
        costs.push_back({});
        
        if (std::find(calls.begin(), calls.end(), c) == calls.end())
        {
            continue;
        }
        // TODO: space wasted in the array

        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            costs[c].push_back(calculate_insertion_cost(c, v, manipulator));
        }
    }

    std::shuffle(calls.begin(), calls.end(), gen);

    while (calls.size() > 0)
    {
        std::vector<int> regret;

        for (auto call : calls)
        {
            auto call_costs = costs[call];
            std::sort(call_costs.begin(), call_costs.end());
            regret.push_back(call_costs[1].first - call_costs[0].first);
        }

        std::vector<std::pair<double, call_id_t>> options;
        for (int i = 0; i < calls.size(); i++)
        {
            options.push_back({-regret[i], calls[i]});
        }

        call_id_t best_call = select_best_geom(options, 0.5).second;        

        vehicle_id_t best_vehicle;
        std::pair<int, std::vector<call_id_t>> best_plan;
        best_plan.first = INT_MAX;
        for (int v = 0; v <= problem.n_vehicles; v++)
        {
            if (costs[best_call][v] < best_plan)
            {
                best_plan = costs[best_call][v];
                best_vehicle = v;
            }
        }

        manipulator.set_plan(best_vehicle, best_plan.second);

        calls.erase(std::remove_if(calls.begin(), calls.end(), [best_call](call_id_t c) {
            return abs(c) == abs(best_call);
        }), calls.end());

        for (auto c : calls)
        {
            costs[c][best_vehicle] = calculate_insertion_cost(c, best_vehicle, manipulator);
        }
    }
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
        manipulator.set_plan(vehicle, calls);
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
        manipulator.set_plan(vehicle, calls);
    }

    return removed_calls;
}

class ReinsertRandomRegret : public BaseOperator
{
public:
    int num_elements;

    ReinsertRandomRegret(SolutionManipulator& manipulator, int num_elements) : BaseOperator(manipulator),
        num_elements(std::min(num_elements, manipulator.solution.problem.get().n_calls)) {}

    void apply() override
    {
        auto removed_calls = remove_calls(manipulator, num_elements);
        regret_insert(removed_calls, manipulator);
    }
};

class ReinsertSimilarRegret : public BaseOperator
{
public:
    int num_elements;

    ReinsertSimilarRegret(SolutionManipulator& manipulator, int num_elements) : BaseOperator(manipulator),
        num_elements(std::min(num_elements, manipulator.solution.problem.get().n_calls)) {}

    void apply() override
    {
        auto removed_calls = remove_similar_vehicles(manipulator, num_elements);
        regret_insert(removed_calls, manipulator);
    }
};

class ReinsertFullRegret : public BaseOperator
{
public:
    int num_elements;

    ReinsertFullRegret(SolutionManipulator& manipulator, int num_elements) : BaseOperator(manipulator),
        num_elements(std::min(num_elements, manipulator.solution.problem.get().n_calls)) {}

    void apply() override
    {
        auto removed_calls = remove_full_vehicles(manipulator, num_elements);
        regret_insert(removed_calls, manipulator);
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

