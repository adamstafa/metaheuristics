#pragma once

#include <memory>
#include <random>
#include <climits>
#include <set>

#include "../problem/solution_manipulator.hpp"
#include "reinsert.hpp"
#include "rng.hpp"
#include "../logger.hpp"


class UCBSampler
{
public:
    int n_actions;
    int steps;
    std::vector<int> counts;
    std::vector<double> means;
    double alpha = 2.0;

    UCBSampler(int n_actions) : n_actions(n_actions), steps(0), counts(n_actions, 0), means(n_actions, 0)
    {
    }

    int sample()
    {
        std::vector<double> scores;
        for (int i = 0; i < n_actions; i++)
        {
            scores.push_back(means[i] + std::sqrt(alpha * std::log(steps) / counts[i]));
        }
        return std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
    }

    void update(int action, double reward)
    {
        counts[action]++;
        means[action] = (means[action] * (counts[action] - 1) + reward) / counts[action];
        steps++;
    }
};

class DiscountedMeanSampler
{
public:
    int n_actions;
    int steps;
    std::vector<int> counts;
    std::vector<double> means;
    std::vector<double> norms;
    double alpha = 1.0;
    double lambda = 0.95;
    double base_score = 0.1;
    double random_action_probability = 0.1;
    int warmup_steps = 100;

    DiscountedMeanSampler(int n_actions) : n_actions(n_actions), steps(0), counts(n_actions, 0), means(n_actions, 0.0), norms(n_actions, 1.0)
    {
    }

    int sample()
    {
        if (std::uniform_real_distribution<>(0.0, 1.0)(gen) < random_action_probability || steps < warmup_steps)
        {
            return gen() % n_actions;
        }

        std::vector<double> scores;
        for (int i = 0; i < n_actions; i++)
        {
            // scores.push_back(means[i] + std::sqrt(alpha * std::log(steps) / counts[i])); // UCB
            scores.push_back(means[i] + base_score); // mean
        }

        // double sum = std::accumulate(scores.begin(), scores.end(), 0.0) ;
        // if (steps % 1000 == 0)
        // {
        //     std::cout << "Scores: ";
        //     for (const auto& score : scores)
        //     {
        //         std::cout << score  / sum << " ";
        //     }
        //     std::cout << std::endl;
        // }
        std::discrete_distribution<int> distribution(scores.begin(), scores.end());
        return distribution(gen);
        // return std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
    }

    void update(int action, double reward)
    {
        counts[action]++;
        double sum = means[action] * norms[action] * lambda + reward;
        norms[action] = norms[action] * lambda + 1;
        means[action] = sum / norms[action];
        steps++;
    }
};

class AdaptiveRandomChoice : public BaseOperator
{
public:
    std::vector<std::unique_ptr<BaseOperator>> operators;
    DiscountedMeanSampler sampler;
    std::set<int> seen_costs;
    int best_cost = INT_MAX;

    AdaptiveRandomChoice(SolutionManipulator& manipulator, std::vector<std::unique_ptr<BaseOperator>> ops) 
        : BaseOperator(manipulator), operators(std::move(ops)), sampler(this->operators.size()), seen_costs()
    {
    }

    void apply() override
    {
        int prev_cost = manipulator.solution.cost();
        auto op = sampler.sample();
        operators[op]->apply();
        int new_cost = manipulator.solution.cost();
        int delta = std::min(new_cost - prev_cost, 0);

        bool seen = seen_costs.find(new_cost) != seen_costs.end();
        int score = 0;

        if (new_cost < best_cost)
        {
            score = 4;
        }
        else if (delta < 0 && !seen)
        {
            score = 2;
        }
        else if (!seen)
        {
            score = 1;
        }
        
        
        seen_costs.insert(new_cost);
        best_cost = std::min(best_cost, new_cost);
        sampler.update(op, score);

        Logger::log("op" + std::to_string(op) + ".delta", delta);
        double mean_sum = std::accumulate(sampler.means.begin(), sampler.means.end(), 0.0);
        for (int i = 0; i < sampler.n_actions; i++)
        {
            Logger::log("op" + std::to_string(i) + ".prob", sampler.means[i] / mean_sum);
        }
        // if (sampler.steps % 1000 == 0)
        // {
        //     std::cout << "Action counts: ";
        //     for (int i = 0; i < sampler.n_actions; i++)
        //     {
        //         std::cout << sampler.counts[i] << (i < sampler.n_actions - 1 ? ", " : "");
                
        //     }
        //     std::cout << std::endl;
        //     std::cout << "Action counts: ";
        //     for (int i = 0; i < sampler.n_actions; i++)
        //     {
        //         std::cout << sampler.means[i] << (i < sampler.n_actions - 1 ? ", " : "");
                
        //     }
        //     std::cout << std::endl;
        // }
    }
};


class IndependentParamsAdaptiveRandomChoice : public BaseOperator
{
public:
    std::vector<std::unique_ptr<BaseReinserter>> operators;
    std::vector<int> sizes;
    DiscountedMeanSampler op_sampler;
    DiscountedMeanSampler size_sampler;
    std::set<int> seen_costs;
    int best_cost = INT_MAX;

    IndependentParamsAdaptiveRandomChoice(SolutionManipulator& manipulator, std::vector<std::unique_ptr<BaseReinserter>> ops, std::vector<int> sizes) 
        : BaseOperator(manipulator), operators(std::move(ops)), sizes(sizes), op_sampler(this->operators.size()), size_sampler(this->sizes.size()), seen_costs()
    {
    }

    void apply() override
    {
        int prev_cost = manipulator.solution.cost();
        
        auto op = op_sampler.sample();
        auto size = size_sampler.sample();

        operators[op]->apply(size);
        int new_cost = manipulator.solution.cost();
        int delta = std::min(new_cost - prev_cost, 0);

        bool seen = seen_costs.find(new_cost) != seen_costs.end();
        int score = 0;

        if (new_cost < best_cost)
        {
            score = 4;
        }
        else if (delta < 0 && !seen)
        {
            score = 2;
        }
        else if (!seen)
        {
            score = 1;
        }


        seen_costs.insert(new_cost);
        best_cost = std::min(best_cost, new_cost);

        op_sampler.update(op, score);
        size_sampler.update(size, score);

        // auto sampler = size_sampler;
        // if (sampler.steps % 1000 == 0)
        // {
        //     std::cout << "size counts: ";
        //     for (int i = 0; i < sampler.n_actions; i++)
        //     {
        //         std::cout << sampler.counts[i] << (i < sampler.n_actions - 1 ? ", " : "");
                
        //     }
        //     std::cout << std::endl;
        //     std::cout << "Action counts: ";
        //     for (int i = 0; i < sampler.n_actions; i++)
        //     {
        //         std::cout << sampler.means[i] << (i < sampler.n_actions - 1 ? ", " : "");
                
        //     }
        //     std::cout << std::endl;
        // }
        // sampler = op_sampler;
        // if (sampler.steps % 1000 == 0)
        // {
        //     std::cout << "operator counts: ";
        //     for (int i = 0; i < sampler.n_actions; i++)
        //     {
        //         std::cout << sampler.counts[i] << (i < sampler.n_actions - 1 ? ", " : "");
                
        //     }
        //     std::cout << std::endl;
        //     // std::cout << "Action counts: ";
        //     // for (int i = 0; i < sampler.n_actions; i++)
        //     // {
        //     //     std::cout << sampler.means[i] << (i < sampler.n_actions - 1 ? ", " : "");
                
        //     // }
        //     // std::cout << std::endl;
        // }
    }
};
