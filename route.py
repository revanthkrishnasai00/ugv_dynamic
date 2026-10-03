"""
Dijkstra's algorithm on a graph of Indian cities.
Build the graph, run Dijkstra, print the shortest path and distance.
"""

import heapq


class Graph:
    """Simple weighted undirected graph stored as an adjacency list."""

    def __init__(self):
        self.adj = {}

    def add_node(self, name):
        self.adj.setdefault(name, {})

    def add_edge(self, city1, city2, distance):
        self.add_node(city1)
        self.add_node(city2)
        self.adj[city1][city2] = distance
        self.adj[city2][city1] = distance  # undirected: road works both ways

    def neighbors(self, city):
        return self.adj.get(city, {})


def dijkstra(graph, start, goal):
    """Returns (path, total_distance) for the shortest route from start to goal."""
    distances = {start: 0}
    previous = {}
    visited = set()
    # min-heap of (distance_so_far, city)
    queue = [(0, start)]

    while queue:
        dist, node = heapq.heappop(queue)

        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            break

        for neighbor, weight in graph.neighbors(node).items():
            new_dist = dist + weight
            if new_dist < distances.get(neighbor, float('inf')):
                distances[neighbor] = new_dist
                previous[neighbor] = node
                heapq.heappush(queue, (new_dist, neighbor))

    if goal not in distances:
        return None, float('inf')

    # reconstruct path by walking backwards through `previous`
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()

    return path, distances[goal]


# ---------------------------------------------------------------------------
# Build the India road graph
# ---------------------------------------------------------------------------

g = Graph()

edges = [
    ("Delhi", "Jaipur", 280), ("Delhi", "Agra", 233), ("Delhi", "Kanpur", 440), ("Delhi", "Chandigarh", 250),
    ("Chandigarh", "Amritsar", 230),
    ("Jaipur", "Ahmedabad", 660), ("Jaipur", "Udaipur", 395),
    ("Udaipur", "Ahmedabad", 260),
    ("Ahmedabad", "Mumbai", 525), ("Ahmedabad", "Surat", 280),
    ("Surat", "Mumbai", 285),
    ("Mumbai", "Pune", 150), ("Mumbai", "Nashik", 165),
    ("Nashik", "Pune", 210), ("Nashik", "Aurangabad", 235),
    ("Pune", "Solapur", 250), ("Pune", "Bangalore", 840),
    ("Aurangabad", "Nagpur", 500),
    ("Solapur", "Hyderabad", 300),
    ("Agra", "Kanpur", 285), ("Agra", "Gwalior", 120),
    ("Gwalior", "Bhopal", 425),
    ("Kanpur", "Lucknow", 90), ("Kanpur", "Varanasi", 320), ("Kanpur", "Bhopal", 555),
    ("Lucknow", "Varanasi", 285), ("Lucknow", "Patna", 530),
    ("Varanasi", "Patna", 245),
    ("Patna", "Kolkata", 580),
    ("Kolkata", "Bhubaneswar", 445),
    ("Bhubaneswar", "Vishakhapatnam", 440),
    ("Bhopal", "Nagpur", 350), ("Bhopal", "Indore", 195),
    ("Indore", "Nagpur", 480),
    ("Nagpur", "Hyderabad", 500), ("Nagpur", "Raipur", 285),
    ("Raipur", "Vishakhapatnam", 500),
    ("Vishakhapatnam", "Vijayawada", 350),
    ("Vijayawada", "Hyderabad", 275), ("Vijayawada", "Chennai", 450),
    ("Hyderabad", "Bangalore", 570), ("Hyderabad", "Chennai", 630),
    ("Bangalore", "Chennai", 350), ("Bangalore", "Mysore", 145),
]

for city1, city2, dist in edges:
    g.add_edge(city1, city2, dist)

# ---------------------------------------------------------------------------
# Run Dijkstra's algorithm
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    start, goal = "Delhi", "Chennai"
    path, total = dijkstra(g, start, goal)

    print(f"Shortest path from {start} to {goal}:")
    print(" -> ".join(path))
    print(f"Total distance: {total} km")