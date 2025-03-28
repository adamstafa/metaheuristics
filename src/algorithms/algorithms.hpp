#pragma once

#include "../problem/solution_manipulator.hpp"
#include "../problem/problem.hpp"
#include "../operators/reinsert.hpp"
#include "../operators/all_permutations.hpp"
#include <cassert>
#include <cmath>

class BaseAlgorithm
{
public:
    ProblemReimagined &problem;
    SolutionManipulator &manipulator;
    BaseOperator &op;

    BaseAlgorithm(ProblemReimagined &problem, SolutionManipulator &manipulator, BaseOperator &op)
        : problem(problem), manipulator(manipulator), op(op){};

    virtual void run(int num_iterations)
    {

    }
};

class LocalSearch : BaseAlgorithm
{
public:
    LocalSearch(ProblemReimagined &problem, SolutionManipulator &manipulator, BaseOperator &op)
        : BaseAlgorithm(problem, manipulator, op) {}

    void run(int num_iterations) override
    {
        for (int i = 0; i < num_iterations; i++)
        {
            int cost = manipulator.solution.cost();

            manipulator.begin();
            op.apply();

            assert(manipulator.solution.feasible());

            if (manipulator.solution.cost() < cost)
            {
                manipulator.commit();
            }
            else
            {
                manipulator.rollback();
            }
        }
    }
};

class SimulatedAnnealing : BaseAlgorithm // TODO: remember the best achieved solution
{
public:
    int warmup_steps;
    Solution best_solution;
    int no_improvement_steps = 0;
    int escape_steps = 5000;

SimulatedAnnealing(ProblemReimagined &problem, SolutionManipulator &manipulator, BaseOperator &op, int warmup_steps)
        : BaseAlgorithm(problem, manipulator, op), warmup_steps(warmup_steps), best_solution(problem) {}

    void run(int num_iterations) override
    {
        int best_cost = manipulator.solution.cost();
        int delta_sum = 0;
        int delta_count = 0;
        for (int i = 0; i < warmup_steps; i++)
        {
            manipulator.begin();

            int old_cost = manipulator.solution.cost();
            op.apply();
            int delta = old_cost - manipulator.solution.cost();

            if (delta < 0)
            {
                manipulator.commit();
                best_cost = std::min(best_cost, manipulator.solution.cost());
            }
            else
            {
                delta_sum += delta;
                delta_count++;

                if (static_cast<float>(rand()) / RAND_MAX < 0.8)
                {
                    manipulator.commit();
                }
                else
                {
                    manipulator.rollback();
                }
            }
        }

        int delta_avg = ((double) delta_sum) / delta_count;

        double initial_temperature = - delta_avg / std::log(0.8);
        double final_temperature = 0.1;
        double alpha = std::pow(final_temperature / initial_temperature, 1.0 / (num_iterations - warmup_steps));

        double temperature = initial_temperature;
        for (int i = warmup_steps; i < num_iterations; i++)
        {
            manipulator.begin();

            int old_cost = manipulator.solution.cost();
            op.apply();
            int delta = manipulator.solution.cost() - old_cost;

            if (manipulator.solution.cost() < best_cost)
            {
                best_cost = manipulator.solution.cost();
                best_solution = manipulator.solution;
                no_improvement_steps = 0;
            }

            if (delta < 0)
            {
                manipulator.commit();
                // no_improvement_steps = 0;
            }
            else if(static_cast<float>(rand()) / RAND_MAX < std::exp(- delta / temperature))
            {
                manipulator.commit();
                no_improvement_steps ++;
            }
            else
            {
                manipulator.rollback();
                no_improvement_steps ++;
            }

            if (no_improvement_steps > escape_steps)
            {
                // TODO: pass escape operator as a parameter
                // ReinsertOperator<FullVehiclesRemover, RandomInserter> escape_op(manipulator, problem.n_calls / 5);
                // BestPermutationOperator best_permutation_op(manipulator);
                ReinsertOperator<RandomRemover, RegretInserter> escape_op(manipulator, 10);

                // std::cout << "Escaping" << std::endl;
                // std::cout << "Before escape: "<< manipulator.solution.cost() << std::endl;
                for (int i = 0; i < 10; i++)
                    escape_op.apply();
                // std::cout << "Mid escape: "<< manipulator.solution.cost() << std::endl;
                // best_permutation_op.apply();
                // std::cout << "After escape: "<< manipulator.solution.cost() << std::endl;
                // std::cout << manipulator.python_string() << std::endl;

                no_improvement_steps = 0;
                best_cost = manipulator.solution.cost();

            }

            temperature *= alpha;

            // if (i % 1000 == 0)
            // {
            //     std::cout << i << "\t" << best_cost << std::endl;
            // }
        }
    }
};
