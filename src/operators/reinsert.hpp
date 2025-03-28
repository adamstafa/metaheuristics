#pragma once

#include <memory>
#include <random>
#include <climits>

#include "../problem/solution_manipulator.hpp"
#include "insert.hpp"
#include "remove.hpp"
#include "rng.hpp"

class BaseOperator
{
public:
    SolutionManipulator& manipulator;

    BaseOperator(SolutionManipulator& manipulator) : manipulator(manipulator)
    {
        srand(time(nullptr));  // TODO: dont initialize random like this
    }

    virtual void apply() = 0;
};


class BaseReinserter : public BaseOperator
{
public:
    BaseReinserter(SolutionManipulator& manipulator) : BaseOperator(manipulator) {}

    virtual void apply(int num_elements) = 0;

};

template <typename RemoverType, typename InserterType>
class ReinsertOperator : public BaseReinserter
{
public:
    RemoverType remover;
    InserterType inserter;
    int num_elements;

    ReinsertOperator(SolutionManipulator& manipulator, int num_elements=0) 
        : BaseReinserter(manipulator),
          inserter(manipulator), 
          remover(manipulator),
          num_elements(std::min(num_elements, manipulator.solution.problem.get().n_calls)){}

    void apply() override
    {
        apply(num_elements);
    }

    void apply(int num_elements) override
    {
        auto removed_calls = remover.remove(num_elements);
        inserter.insert(std::move(removed_calls));
    }
};

using ReinsertRandomRegret = ReinsertOperator<RandomRemover, RegretInserter>;
using ReinsertSimilarRegret = ReinsertOperator<SimilarVehiclesRemover, RegretInserter>;
using ReinsertFullRegret = ReinsertOperator<FullVehiclesRemover, RegretInserter>;

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
