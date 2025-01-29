import sys
import heapq

class Graph:
    def __init__(self, num_vertices):
        """
        Initializes the graph with a list of edges.
        """
        self.num_vertices = num_vertices
        self.edges = []

    def add_edge(self, u, v, weight):
        """
        Adds an edge to the graph.
        """
        self.edges.append((u, v, weight))


def bellman_ford(graph, start):
    num_vertices = graph.num_vertices
    distance = [float('inf')] * num_vertices
    distance[start] = 0

    # Relax edges up to (num_vertices - 1) times
    for _ in range(num_vertices - 1):
        updated = False
        for u, v, weight in graph.edges:
            if distance[u] != float('inf') and distance[u] + weight < distance[v]:
                distance[v] = distance[u] + weight
                updated = True
        if not updated:
            break

    # Check for negative cycles
    for u, v, weight in graph.edges:
        if distance[u] != float('inf') and distance[u] + weight < distance[v]:
            return None  # Negative cycle detected

    return distance

def dijkstra(graph, start):
    num_vertices = graph.num_vertices
    distances = [float('inf')] * num_vertices
    distances[start] = 0

    # Priority queue (min-heap) to store (distance, vertex) pairs
    priority_queue = [(0, start)]

    while priority_queue:
        # Pop the vertex with the smallest distance
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # If the current distance is greater than the recorded distance, skip
        if current_distance > distances[current_vertex]:
            continue

        # Explore neighbors
        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight

            # If a shorter path is found, update the distance and push to the priority queue
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances


input_data = """
4 6
0 1 -0.5
0 2 1
1 3 2
2 3 1
3 0 -1
3 1 -0.5                                                            b
"""

# Parse input data
lines = input_data.strip().split("\n")
num_vertices, num_edges = map(int, lines[0].split())
graph = Graph(num_vertices)

for line in lines[1:]:
    u, v, weight = map(float, line.split())
    graph.add_edge(int(u), int(v), weight)

# Run Bellman-Ford from vertex 0
start_vertex = 0
distances = bellman_ford(graph, start_vertex)

# Output the result
if distances is None:
    print("negativen cikel")
else:
    print("Shortest distances from vertex", start_vertex, ":")
    for vertex, distance in enumerate(distances):
        print(f"Vertex {vertex}: {distance}")
