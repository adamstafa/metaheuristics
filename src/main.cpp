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


int main(int argc, char* argv[])
{
    // if (argc != 5) {
    //     std::cerr << "Usage: " << argv[0] << " <instance> <weight1> <weight2> <weight3>" << std::endl;
    //     return 1;
    // }

    // std::string problem_path = argv[1];
    // std::vector<double> weights {std::strtod(argv[2], nullptr), std::strtod(argv[3], nullptr), std::strtod(argv[4], nullptr)};

    std::string problem_path = "../data/Call_300_Vehicle_90.txt";
    std::vector<double> weights {0.25, 0.6, 0.15};
    // optimal 0.25 0.6 0.15

    // auto parsed = parse_problem("../data/Call_7_Vehicle_3.txt");
    // auto parsed = parse_problem("../data/Call_18_Vehicle_5.txt");
    // auto parsed = parse_problem("../data/Call_35_Vehicle_7.txt");
    // auto parsed = parse_problem("../data/Call_80_Vehicle_20.txt");
    // auto parsed = parse_problem("../data/Call_130_Vehicle_40.txt");
    // auto parsed = parse_problem("../data/Call_300_Vehicle_90.txt");

    auto parsed = parse_problem(problem_path);

    auto problem = ProblemReimagined(parsed);

    auto sol = Solution(problem);
    SolutionManipulator manipulator(sol);

    std::vector<std::unique_ptr<BaseOperator>> random_operators;
    random_operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 20));
    random_operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 10));
    random_operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 3));
    // random_operators.emplace_back(std::make_unique<ReinsertRandomRegret>(manipulator, 1));
    auto select_random = std::make_unique<RandomChoice>(manipulator, std::move(random_operators));

    std::vector<std::unique_ptr<BaseOperator>> similar_operators;
    similar_operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 20));
    similar_operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 10));
    similar_operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 3));
    // similar_operators.emplace_back(std::make_unique<ReinsertSimilarRegret>(manipulator, 1));
    auto select_similar = std::make_unique<RandomChoice>(manipulator, std::move(similar_operators));

    std::vector<std::unique_ptr<BaseOperator>> full_operators;
    full_operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 20));
    full_operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 10));
    full_operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 3));
    // full_operators.emplace_back(std::make_unique<ReinsertFullRegret>(manipulator, 1));
    auto select_full = std::make_unique<RandomChoice>(manipulator, std::move(full_operators));

    std::vector<std::unique_ptr<BaseOperator>> sa_operators;
    sa_operators.emplace_back(std::move(select_random));
    sa_operators.emplace_back(std::move(select_similar));
    sa_operators.emplace_back(std::move(select_full));
    WeightedRandomChoice sa_operator(manipulator, std::move(sa_operators), weights);


    SimulatedAnnealing sa(problem, manipulator, sa_operator, 100);
    sa.run(10000);


    std::cout << sol.feasible() << std::endl;
    std::cout << sol.valid() << std::endl;
    std::cout << sol.cost() << std::endl << std::endl;
    std::cout << manipulator.python_string() << std::endl;

    return 0;
}
