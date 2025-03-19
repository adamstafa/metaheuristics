## Description of Operators

All operators are based on removing several calls at the same time and reinserting them.
The operators remove 3, 10 or 20 calls (the number is selected randomly).
Then they reinsert the calls one by one using the 2-regret heuristic with small probability of selecting a sub-optimal call.
The insertion cost is determined from all possible options of inserting a call in all vehicles.

### Operator 1

Removes calls completely randomly.
We might want to remove every call at some point and using exclusively heuristic approaches could prevent certain calls from being removed.
The optimal weight for this operator was 0.25.


### Operator 2

Removes similar calls according to the similarity score based on relatedness of cargo, time, distance, and compatibility.
The idea is that the selected calls should be mutually interchangeable and easy to swap.
The optimal weight for this operator was 0.6.

### Operator 3

Removes calls from vehicles with high number of calls.
The idea is to free up their schedule so new calls can be inserted in them.
This should prevent premature convergence and open up options for the other operators.
The optimal weight for this operator was 0.15.
