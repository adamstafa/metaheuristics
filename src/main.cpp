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
#include "logger.hpp"
#include "operators/remove.hpp"

AdaptiveRandomChoice get_alns_operator(SolutionManipulator& manipulator)
{
    std::vector<std::unique_ptr<BaseOperator>> operators;
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 40));
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 30));
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 20));
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 10));
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 5));
    operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 1));

    operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 30));
    operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 20));
    
    operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 20));

    operators.emplace_back(std::make_unique<ReinsertExpensiveRegret>(manipulator, 20));

    AdaptiveRandomChoice op(manipulator, std::move(operators));
    return op;
}

int main(int argc, char* argv[])
{
    std::string log_path = "data.csv";
    int seconds = 5;
    if (argc == 4) {
        log_path = argv[2];
        seconds = std::stoi(argv[3]);
    }

    std::string problem_path = argv[1];

    // std::string problem_path = "../data/Call_7_Vehicle_3.txt";
    // std::string problem_path = "../data/Call_18_Vehicle_5.txt";
    // std::string problem_path = "../data/Call_35_Vehicle_7.txt";
    // std::string problem_path = "../data/Call_80_Vehicle_20.txt";
    // std::string problem_path = "../data/Call_130_Vehicle_40.txt";
    // std::string problem_path = "../data/Call_300_Vehicle_90.txt";

    // std::vector<double> weights {std::strtod(argv[2], nullptr), std::strtod(argv[3], nullptr), std::strtod(argv[4], nullptr), std::strtod(argv[5], nullptr)};
    std::vector<double> weights {0.3, 0.15, 0.3, 0.25}; 

    SimilarVehiclesRemover::cargo_weight = weights[0];
    SimilarVehiclesRemover::distance_weight = weights[1];
    SimilarVehiclesRemover::time_weight = weights[2];
    SimilarVehiclesRemover::compatibility_weight = weights[3];

    auto parsed = parse_problem(problem_path);
    auto problem = ProblemReimagined(parsed);
    auto sol = Solution(problem);
    SolutionManipulator manipulator(sol);

    auto alns_operator = get_alns_operator(manipulator);

    // SimulatedAnnealing algo(problem, manipulator, alns_operator, 100);
    // algo.run(10000);

    RecordToRecord algo(problem, manipulator, alns_operator, [&]() { std::cerr << problem_path << ": " <<  algo.best_solution.cost() << std::endl; });
    algo.run(seconds);

    auto best_sol = algo.best_solution;

    // std::cout << best_sol.feasible() << std::endl;
    // std::cout << best_sol.valid() << std::endl;
    // std::cout << best_sol.cost() << std::endl;
    std::cout << best_sol.python_string() << std::endl;

    Logger::dump(log_path);

    return 0;
}
