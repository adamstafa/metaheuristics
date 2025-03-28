#include <iostream>
#include <memory>
#include <cassert>
#include <string>
#include <cstdlib>

#include "problem/parser.hpp"
#include "problem/problem.hpp"
#include "problem/problem_reimagined.hpp"
#include "problem/solution.hpp"
#include "problem/solution_manipulator.hpp"
#include "operators/reinsert.hpp"
#include "algorithms/algorithms.hpp"
#include "operators/alns_operator.hpp"

AdaptiveRandomChoice get_alns_operator(SolutionManipulator& manipulator)
{
    std::vector<std::unique_ptr<BaseOperator>> operators;
    // operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 30));
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 20));
    // operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 10));
    // operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 3));
    // operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 40));
    operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 30));
    operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 20));
    operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 10));
    operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 3));
    // operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 30));
    operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 20));
    // operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 10));
    // operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 3));

    AdaptiveRandomChoice op(manipulator, std::move(operators));
    return op;
}

IndependentParamsAdaptiveRandomChoice get_independent_params_alns_operator(SolutionManipulator& manipulator)
{
    std::vector<std::unique_ptr<BaseReinserter>> operators;
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator));
    operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator));
    operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator));
    std::vector<int> sizes = {3, 10, 20};

    IndependentParamsAdaptiveRandomChoice op(manipulator, std::move(operators), sizes);
    return op;
}

WeightedRandomChoice get_fixed_weights_operator(SolutionManipulator& manipulator, std::vector<double> weights)
{
    std::vector<std::unique_ptr<BaseOperator>> random_operators;
    random_operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 20));
    random_operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 10));
    random_operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 3));
    auto select_random = std::make_unique<RandomChoice>(manipulator, std::move(random_operators));

    std::vector<std::unique_ptr<BaseOperator>> similar_operators;
    similar_operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 20));
    similar_operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 10));
    similar_operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 3));
    auto select_similar = std::make_unique<RandomChoice>(manipulator, std::move(similar_operators));

    std::vector<std::unique_ptr<BaseOperator>> full_operators;
    full_operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 20));
    full_operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 10));
    full_operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 3));
    auto select_full = std::make_unique<RandomChoice>(manipulator, std::move(full_operators));

    std::vector<std::unique_ptr<BaseOperator>> sa_operators;
    sa_operators.emplace_back(std::move(select_random));
    sa_operators.emplace_back(std::move(select_similar));
    sa_operators.emplace_back(std::move(select_full));

    WeightedRandomChoice random_choice(manipulator, std::move(sa_operators), weights);
    return random_choice;
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Wrong number of arguments" << std::endl;
        return 1;
    }

    std::string problem_path = argv[1];

    // std::string problem_path = "../data/Call_7_Vehicle_3.txt";
    // std::string problem_path = "../data/Call_18_Vehicle_5.txt";
    // std::string problem_path = "../data/Call_35_Vehicle_7.txt";
    // std::string problem_path = "../data/Call_80_Vehicle_20.txt";
    // std::string problem_path = "../data/Call_130_Vehicle_40.txt";
    // std::string problem_path = "../data/Call_300_Vehicle_90.txt";

    // std::vector<double> weights {std::strtod(argv[2], nullptr), std::strtod(argv[3], nullptr), std::strtod(argv[4], nullptr)};
    // std::vector<double> weights {0.25, 0.6, 0.15};
    // optimal 0.25 0.6 0.15

    auto parsed = parse_problem(problem_path);
    auto problem = ProblemReimagined(parsed);
    auto sol = Solution(problem);
    SolutionManipulator manipulator(sol);


    auto sa_operator = get_alns_operator(manipulator);
    // auto sa_operator = get_independent_params_alns_operator(manipulator);

    SimulatedAnnealing sa(problem, manipulator, sa_operator, 100);
    sa.run(10000);

    auto best_sol = sa.best_solution;

    // std::cout << best_sol.feasible() << std::endl;
    // std::cout << best_sol.valid() << std::endl;
    // std::cout << best_sol.cost() << std::endl << std::endl;
    std::cout << manipulator.python_string() << std::endl;

    return 0;
}
