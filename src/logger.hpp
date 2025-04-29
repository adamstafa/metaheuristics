#pragma once

#include <iostream>
#include <vector>
#include <string>

class Logger {
private:
    struct LogEntry {
        int iteration;
        std::string label;
        double value;
    };

    static std::vector<LogEntry> log_entries;
    static int iteration;

    Logger() {} // Private constructor to prevent instantiation

public:
    static void log(const std::string& label, double value) {
        // Logging disabled for final submission since it takes too much memory
        // TODO: write entries to the file and don't store them for the whole duration of the run
        // log_entries.push_back({iteration, label, value});
    }

    static void advance_iteration() {
        ++iteration;
    }

    static void dump(const std::string& path) {
        std::ofstream file(path);
        if (!file.is_open()) {
            throw std::ios_base::failure("Failed to open file: " + path);
        }

        file << "iteration,label,value\n";
        for (const auto& entry : log_entries) {
            file << entry.iteration << "," << entry.label << "," << entry.value << "\n";
        }

        file.close();
    }
};

// Define static members
std::vector<Logger::LogEntry> Logger::log_entries;
int Logger::iteration = 0;
