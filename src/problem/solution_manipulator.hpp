#pragma once

#include <memory>
#include "solution.hpp"

class ManipulatorFrame
{
public:
    
    Solution solution;
    std::vector<std::vector<call_id_t>> calls;

    ManipulatorFrame(Solution solution, std::vector<std::vector<call_id_t>> calls) : solution(solution), calls(calls)
    {};
};


class SolutionManipulator
{
public:
    Solution& solution;
    std::vector<std::vector<call_id_t>> calls;
    std::vector<ManipulatorFrame> frames;

    SolutionManipulator(Solution& solution) : solution(solution), calls(solution.problem.get().n_vehicles + 1), frames()
    {
        for (int i = 1; i <= solution.problem.get().n_calls; i++)
        {
            calls[0].push_back(i);
            calls[0].push_back(-i);
        }
    };

    void set_plan(vehicle_id_t vehicle, std::vector<call_id_t>::const_iterator begin, std::vector<call_id_t>::const_iterator end)
    {
        calls[vehicle].assign(begin, end);
        if (vehicle == 0)
        {
            return;
        }

        solution.clone_vehicle_solution(vehicle);
        solution.set_vehicle_plan(vehicle, begin, end);
    }

    void commit()
    {
        frames.pop_back();
    }

    void begin()
    {
        frames.push_back({solution, calls});
    }

    void rollback()
    {
        solution = frames.back().solution; // TODO: this should be just pop
        calls = frames.back().calls;
        frames.pop_back();
    }

    std::string python_string()
    {
        std::vector<int> joined_calls;
        for (size_t i = 1; i < calls.size(); ++i)
        {
            joined_calls.insert(joined_calls.end(), calls[i].begin(), calls[i].end());
            joined_calls.push_back(0);
        }
        joined_calls.insert(joined_calls.end(), calls[0].begin(), calls[0].end());

        std::string result = "[";
        for (size_t i = 0; i < joined_calls.size(); ++i)
        {
            result += std::to_string(abs(joined_calls[i]));
            if (i != joined_calls.size() - 1)
            {
                result += ", ";
            }
        }
        result += "]";
        return result;
    }

    vehicle_id_t get_vehicle_for_call(call_id_t call)
    {
        for (vehicle_id_t v = 0; v <= solution.problem.get().n_vehicles; v++)
        {
            if (std::find(calls[v].begin(), calls[v].end(), call) != calls[v].end())
            {
                return v;
            }
        }
        return -1;
    }
};
