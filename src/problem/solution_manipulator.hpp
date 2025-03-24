#pragma once

#include <memory>
#include "solution.hpp"

class ManipulatorFrame
{
public:
    
    Solution solution;
    std::vector<std::vector<int>> calls;

    ManipulatorFrame(Solution solution, std::vector<std::vector<int>> calls) : solution(solution), calls(calls)
    {};
};


class SolutionManipulator
{
public:
    Solution& solution;
    std::vector<std::vector<int>> calls;
    std::vector<ManipulatorFrame> frames;

    SolutionManipulator(Solution& solution) : solution(solution), calls(solution.problem.get().n_vehicles + 1), frames()
    {
        for (int i = 1; i <= solution.problem.get().n_calls; i++)
        {
            calls[0].push_back(i);
            calls[0].push_back(-i);
        }
    };

    void set_plan(vehicle_id_t vehicle, std::vector<call_id_t> plan)
    {
        // for (auto call : plan)
        // {
        //     assert(call > 0 && call <= solution.problem.get().n_calls);
        // }

        calls[vehicle] = plan;
        if (vehicle == 0)
        {
            return;
        }

        solution.clone_vehicle_solution(vehicle);

        // TODO: optimize - no need to remove everything, maybe we can just save VehicleSolution and restore the rest of variables...
        // TODO: I don't understand the previous comment lol

        // int same = 0;
        // auto& vs = solution.vehicle_solution(vehicle);
        // while (same < plan.size() && same < vs.num_calls() && vs.plan[same + 1].call == plan[same])
        // {
        //     same++;
        // }

        // while (solution.vehicle_solution(vehicle).num_calls() > same)
        // {
        //     solution.pop_call(vehicle);
        // };
        // for (int i = same; i < plan.size(); i++)
        // {
        //     solution.push_call(vehicle, plan[i]);
        // }
        solution.set_vehicle_plan(vehicle, plan);
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
