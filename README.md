# Average Distance in a Directed Weighted Graph

This repository contains an implementation to calculate the average distance between all pairs of vertices in a directed weighted graph. The graph may contain negative edge weights, and the algorithm handles cases where negative cycles are present.

## Problem Description

Given a directed and weighted graph \( G = (V(G), E(G)) \) with edge weights \( w : E(G) \rightarrow \mathbb{R} \) (which may be negative), the task is to calculate the average distance \( \mu(G) \) between all pairs of vertices. The average distance is defined as:
\[
\mu(G) = \frac{\sum_{(u, v) \in V(G) \times V(G)} d_G(u, v)}{n(n - 1)},
\]
where \( n = |V(G)| \), and \( d_G(u, v) \) represents the shortest path distance from vertex \( u \) to vertex \( v \) in the graph \( G \).

If the graph contains a negative cycle, the algorithm should return `negativen cikel` (Slovene for "negative cycle").

## Input Format

- The first line contains the integers \( n \) and \( m \), representing the number of vertices and edges in the graph.
- The following \( m \) lines each contain three values: \(u_i,\ v_i, \ w(u_i, v_i)\), representing a directed edge from vertex \(u_i\) to vertex \(v_i\) with weight \( w(u_i, v_i)\).

*Note*: The weights \( w(u_i, v_i)\) are given with a precision of two decimal places.

### Input constraints:
- \( 1 \leq n \leq 1000 \)
- \( 1 \leq m \leq 5n \)
- \( 0 \leq ui, vi \leq n - 1 \)
- \( -2 \leq w(ui, vi) \leq 10 \)

## Output Format

- If the graph contains a negative cycle, the output should be `negativen cikel`.
- Otherwise, the output should be the average distance \( \mu(G) \) calculated to a precision of at most \( 10^{-5} \).




## Implementation Details

The algorithm follows these steps:
1. Graph Construction:
   - Add a new vertex \(v0\)​ to the graph G and connect it to all other vertices with edges of weight 0, resulting in a new graph G′.
   - Calculate the shortest path distances \(f_v=d_{G′}(v0,v)\) for every vertex \(v\in V(G)\) using the Bellman-Ford algorithm.
2. Edge Weight Transformation:
   - Transform the edge weights using the formula:
        \[ w′(e)=w(e)+f_u−f_v\]
        where \(e=uv\) is an edge in the graph. This ensures that all edge weights in the transformed graph are non-negative.
3. Shortest Path Calculation:
   - Use Dijkstra's algorithm on the transformed graph G′G′ to calculate the shortest path distances dG′(u,v)dG′​(u,v) for all pairs of vertices u,vu,v.

4. Average Distance Calculation:
   - Calculate the average distance  \( \mu(G) \) using the formula:
     \[
      \mu(G) = \frac{\sum_{(u, v) \in V(G) \times V(G)} d_G(u, v)}{n(n - 1)},
      \]
     where \(d_G(u,v)\) is derived from the transformed graph's distances.

5. Negative Cycle Detection:
   - If the Bellman-Ford algorithm detects a negative cycle during the initial step, the algorithm immediately returns "negativen cikel".

## Code structure
1. Graph Class:
  - Manages vertices and edges.
  - Adds vertices and connects them via edges.

2. Bellman-Ford Algorithm:
  - Computes the shortest paths from the added vertex v0v0​ to all other vertices.
  - Detects negative cycles.

3. Dijkstra's Algorithm:
  - Computes the shortest paths in the transformed graph with non-negative edge weights.

4. Main Function:
  - Reads input data.
  - Builds the graph and performs the necessary transformations.
  - Computes the average distance or detects negative cycles.
  - Outputs the result.

## Usage
Provide an input through standard input in the specified format. The code will output the average distance or `negativen cikel` if a negative cycle is detected.

Example Input:
```
4 6
0 1 -0.5
0 2 1
1 3 2
2 3 1
3 0 -1
3 1 -0.5
```

Example Output:
```
0.4166666666666667
```
## Testing
To test the solution:

1. Place `solution.py` (the solution file) in the same folder as `test_runner.py` and the Tests folder.

2. Run the command:
```
python3 test_runner.py -script solution.py -test_nb 1
```
to test a single test case, or:
```
python3 test_runner.py -script solution.py
```
to test all the test cases in the Tests folder.
