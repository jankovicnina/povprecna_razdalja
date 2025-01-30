import sys
import heapq

class Graph:
    def __init__(self, num_vertices):
        """
        Initializes the graph with an adjacency list.
        """
        self.num_vertices = num_vertices
        self.adjacency_list = [[] for _ in range(num_vertices)]

    def add_edge(self, u, v, weight):
        """
        Adds a directed edge to the graph.
        """
        self.adjacency_list[u].append((v, weight))


def bellman_ford(graph, start):
    num_vertices = graph.num_vertices
    distance = [float('inf')] * num_vertices
    distance[start] = 0

    # Relax edges up to (num_vertices - 1) times
    for _ in range(num_vertices - 1):
        updated = False
        for u in range(num_vertices):
            for v, weight in graph.adjacency_list[u]:
                if distance[u] != float('inf') and distance[u] + weight < distance[v]:
                    distance[v] = distance[u] + weight
                    updated = True
        if not updated:
            break

    # Check for negative cycles
    for u in range(num_vertices):
        for v, weight in graph.adjacency_list[u]:
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
        for neighbor, weight in graph.adjacency_list[current_vertex]:
            distance = current_distance + weight

            # If a shorter path is found, update the distance and push to the priority queue
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances


def calculate_average_distance(n, edges):
    # Create the graph with n+1 vertices (including v0)
    graph = Graph(n + 1)
    
    # Add the original edges
    for u, v, w in edges:
        graph.add_edge(u, v, w)
    
    # Connect new vertex v0 to all other vertices with undirected edges of weight 0
    v0 = n
    for u in range(n):
        graph.add_edge(v0, u, 0)  # Edge from v0 to u
    
    # Run Bellman-Ford from v0
    distances = bellman_ford(graph, v0)
    if distances is None:
        return "negativen cikel"
    
    # Re-weight the edges
    reweighted_graph = Graph(n)
    for u in range(n):
        for v, weight in graph.adjacency_list[u]:
            if v != v0:  # Skip edges involving v0
                reweighted_graph.add_edge(u, v, weight + distances[u] - distances[v])
    
    # Calculate all pairs shortest paths using Dijkstra
    total_distance = 0
    for u in range(n):
        dist = dijkstra(reweighted_graph, u)
        for v in range(n):
            if u != v:
                if dist[v] == float('inf'):
                    return "negativen cikel"
                # Adjust the distance back to the original graph's distance
                total_distance += dist[v] - distances[u] + distances[v]
    
    # Calculate the average distance
    average_distance = total_distance / (n * (n - 1))
    return average_distance

if __name__ == "__main__":
    # Parse input data
    lines = sys.stdin.read().strip().split("\n")
    num_vertices, num_edges = map(int, lines[0].split())
    edges = []
    for line in lines[1:]:
        u, v, weight = map(float, line.split())
        edges.append((int(u), int(v), weight))

    # Calculate average distance
    result = calculate_average_distance(num_vertices, edges)
    print(result)