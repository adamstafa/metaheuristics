## Description of the Algorithm

The algorithm is based on the ALNS framework with the record-to-record travel acceptance criterion.
A new solution is accepted if its cost is less than $z^B + D$ where $z^B$ is the best cost achieved so far and $D = 0.005\frac{T-t}{T}z^B$ where $t$ is the current running time and $T$ is the time limit.
For the exam run, I will use $D = 0.1\frac{T-t}{T}z^B$ to lower the risk of getting stuck in local optima in the larger instances.

The operators used in the algorithm are all reinsertions of up to 40 calls.
Several heuristics are used to remove the calls: random, similarity score clustering, vehicles with many calls, and calls in expensive positions.
While the heuristics help a bit, using only the random removal provides very similar results that are only marginally worse.
The calls are then inserted according to the regret heuristic with a small probability of selecting a suboptimal call.
