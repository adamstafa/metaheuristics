## Description of Operators

The operators are taken from the previous assignment.
They remove 3, 10, 20, or 30 calls at once and reinsert them one by one using the 2-regret heuristic with small probability of selecting a sub-optimal call.
The insertion cost is determined from all possible options of inserting a call in all vehicles.
The adaptive heuristic selects both the operator and the number of calls.

### Operator 1

Removes calls completely randomly.
We might want to remove every call at some point and using exclusively heuristic approaches could prevent certain calls from being removed.


### Operator 2

Removes similar calls according to the similarity score based on relatedness of cargo, time, distance, and compatibility.
The idea is that the selected calls should be mutually interchangeable and easy to swap.

### Operator 3

Removes calls from vehicles with high number of calls.
The idea is to free up their schedule so new calls can be inserted in them.
This should prevent premature convergence and open up options for the other operators.
