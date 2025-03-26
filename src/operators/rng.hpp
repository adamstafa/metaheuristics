#pragma once

#include <random>

std::random_device rd;
std::mt19937 gen(rd());
