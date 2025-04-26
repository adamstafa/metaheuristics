#pragma once

#include "../problem/solution_manipulator.hpp"
#include "../problem/problem.hpp"
#include "../operators/reinsert.hpp"
#include "../operators/all_permutations.hpp"
#include "../logger.hpp"
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
    int escape_steps = 10000000;

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
            Logger::advance_iteration();
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
                no_improvement_steps = 0;
            }

            if (manipulator.solution.cost() < best_solution.cost())
            {
                best_solution = manipulator.solution;
            }

            if (delta < 0)
            {
                manipulator.commit();
                no_improvement_steps = 0;
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
                TryReinsertAll escape_op {manipulator};

                std::cout << "Escaping" << std::endl;
                std::cout << "Before escape: "<< manipulator.solution.cost() << std::endl;
                // for (int i = 0; i < 10; i++)
                escape_op.apply();
                // std::cout << "Mid escape: "<< manipulator.solution.cost() << std::endl;
                // best_permutation_op.apply();
                std::cout << "After escape: "<< manipulator.solution.cost() << std::endl;
                // std::cout << manipulator.python_string() << std::endl;

                no_improvement_steps = 0;
                best_cost = manipulator.solution.cost();

            }

            temperature *= alpha;

            Logger::log("cost", manipulator.solution.cost());
            Logger::log("temperature", temperature);
            Logger::log("best_cost", best_cost);
            Logger::log("no_improvement_steps", no_improvement_steps);
            Logger::log("delta", delta);
            if (delta > 0)
            {
                Logger::log("acceptance_probability", std::exp(- delta / temperature));
            }
            Logger::advance_iteration();

            // if (i % 1000 == 0)
            // {
            //     std::cout << i << "\t" << best_cost << std::endl;
            // }
        }
    }
};

class RecordToRecord : BaseAlgorithm
{
public:
    Solution best_solution;

    RecordToRecord(ProblemReimagined &problem, SolutionManipulator &manipulator, BaseOperator &op)
        : BaseAlgorithm(problem, manipulator, op), best_solution(problem) {}

    void run(int num_iterations) override
    {
        int best_cost = manipulator.solution.cost();

        for (int i = 0; i < num_iterations; i++)
        {
            manipulator.begin();

            int old_cost = manipulator.solution.cost();

            op.apply();
            int delta = manipulator.solution.cost() - old_cost;
            double d = 0.05 * (1.0 - ((double) i) / num_iterations) * best_cost;
            int acceptance_threshold = best_cost + d;

            if (manipulator.solution.cost() < best_cost)
            {
                best_cost = manipulator.solution.cost();
                best_solution = manipulator.solution;
            }

            if (manipulator.solution.cost() < acceptance_threshold)
            {
                manipulator.commit();
            }
            else
            {
                manipulator.rollback();
            }
            
            
            Logger::log("cost", manipulator.solution.cost());
            Logger::log("temperature", 0.0);
            Logger::log("best_cost", best_cost);
            Logger::log("delta", delta);
            if (delta > 0)
            {
                Logger::log("acceptance_probability", 0.0);
            }
            Logger::advance_iteration();
        }
    }
};
