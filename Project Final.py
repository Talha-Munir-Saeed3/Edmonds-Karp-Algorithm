import collections
from collections import deque
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# Define a Graph class to represent a flow network
class Graph:
    def __init__(self, V):
        self.V = V
        self.capacity = np.zeros((V, V), dtype=int)
        self.flow = np.zeros((V, V), dtype=int)
        self.adj = [[] for _ in range(V)]
        self.augmenting_paths = []
        self.path_flows = []

    # Method to add an edge with a specific capacity
    def add_Edge(self, u, v, capacity):
        self.capacity[u][v] = capacity  
        self.adj[u].append(v)           
        self.adj[v].append(u)           
    # Breadth-First Search to find an augmenting path
    def bfs(self, s, t, parent):
        # Initialize parent list to track the path
        parent[:] = [-1] * self.V
        parent[s] = -2  
        # Queue for BFS, storing node and path flow capacity
        q = deque([(s, float('inf'))])
        current_path = [s]  

        while q:
            u, flow_in_path = q.popleft()  # Dequeue a vertex

            # Explore all neighbors of u
            for v in self.adj[u]:
                residual_capacity = self.capacity[u][v] - self.flow[u][v]
                # If there's residual capacity and v is not visited
                if parent[v] == -1 and residual_capacity > 0:
                    parent[v] = u  
                    current_path.append(v)
                    new_flow = min(flow_in_path, residual_capacity)  # Find bottleneck flow
                    if v == t:  
                        self.augmenting_paths.append(current_path.copy()) 
                        self.path_flows.append(new_flow) 
                        return new_flow
                    q.append((v, new_flow))
        return 0  # Return 0 if no augmenting path exists

    # Depth-First Search to find reachable vertices for min-cut
    def dfs(self, u, visited):
        visited[u] = True
        for v in self.adj[u]:
            if not visited[v] and self.capacity[u][v] - self.flow[u][v] > 0:
                self.dfs(v, visited)

    # Edmonds-Karp algorithm to find the maximum flow
    def max_Flow(self, s, t):
        total_flow = 0  
        parent = [-1] * self.V  
        self.augmenting_paths = [] 
        self.path_flows = []  

        # While there's an augmenting path
        while (flow_in_path := self.bfs(s, t, parent)) > 0:
            total_flow += flow_in_path  # Add path flow to total flow
            v = t
            # Update residual capacities in the flow network
            while v != s:
                u = parent[v]
                self.flow[u][v] += flow_in_path
                self.flow[v][u] -= flow_in_path
                v = u
        return total_flow

    # Method to find edges in the min-cut
    def min_Cut(self, s):
        visited = [False] * self.V  
        self.dfs(s, visited) 
        min_cut_edges = []
        # Identify edges crossing the cut
        for u in range(self.V):
            if visited[u]:
                for v in self.adj[u]:
                    if not visited[v] and self.capacity[u][v] > 0:
                        min_cut_edges.append((u, v))
        return min_cut_edges

# Subclass to add visualization capabilities to the Graph
class GraphVisual(Graph):
    def plot_graph(self, title, s=None, t=None, augmenting_paths=None, path_flows=None, path_index=None, min_cut_edges=None, show_flow=False, max_flow=None):
        plt.figure(figsize=(14, 8))  # Set plot size
        G = nx.DiGraph()  # Create a directed graph

        # Add nodes to the graph
        G.add_nodes_from(range(self.V))
        
        # Add edges with capacity and flow labels
        for u in range(self.V):
            for v in self.adj[u]:
                if self.capacity[u][v] > 0:
                    capacity = self.capacity[u][v]
                    flow = self.flow[u][v]
                    if show_flow:
                        G.add_edge(u, v, weight=f"{flow}/{capacity}")
                    else:
                        G.add_edge(u, v, weight=f"{capacity}")

        pos = nx.circular_layout(G)  # Layout for visualization

        # Draw nodes and edges
        nx.draw_networkx_nodes(G, pos, node_size=700, node_color="lightblue")
        nx.draw_networkx_edges(G, pos, edge_color="gray", arrows=True, arrowsize=20)

        # Highlight source and sink nodes
        if s is not None:
            nx.draw_networkx_nodes(G, pos, nodelist=[s], node_color="green", node_size=700)
        if t is not None:
            nx.draw_networkx_nodes(G, pos, nodelist=[t], node_color="red", node_size=700)

        # Highlight augmenting paths
        if augmenting_paths:
            for i, path in enumerate(augmenting_paths):
                if path_index is None or i == path_index:
                    path_edges = list(zip(path[:-1], path[1:]))
                    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="green", width=3, arrows=True)
                    if path_flows and i < len(path_flows):
                        path_text = f"Path: {' → '.join(map(str, path))}\nPath Flow: {path_flows[i]}"
                        plt.text(0.02, 0.95, path_text, transform=plt.gca().transAxes, fontsize=10,
                                 verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Highlight min-cut edges
        if min_cut_edges:
            nx.draw_networkx_edges(G, pos, edgelist=min_cut_edges, edge_color="red", style="dashed", width=2)

        # Draw labels
        nx.draw_networkx_labels(G, pos, font_size=12, font_color="black")
        edge_labels = nx.get_edge_attributes(G, "weight")
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10)

        # Add title and max flow information
        plt.title(title)
        if max_flow is not None:
            plt.title(f"{title}\nMax Flow: {max_flow}")
        plt.axis("off")
        plt.tight_layout()
        plt.show()

# Main program to demonstrate max flow, augmenting paths, and min-cut
if __name__ == "__main__":
    # Create a graph with 10 nodes
    g = GraphVisual(10)
    g.add_Edge(0, 1, 3)
    g.add_Edge(0, 3, 4)
    g.add_Edge(0, 2, 2)
    g.add_Edge(1, 4, 5)
    g.add_Edge(2, 5, 4)
    g.add_Edge(3, 7, 2)
    g.add_Edge(5, 4, 4)
    g.add_Edge(5, 8, 1)
    g.add_Edge(7, 8, 3)
    g.add_Edge(4, 9, 3)
    g.add_Edge(8, 9, 5)
    g.add_Edge(5,6,2)
    g.add_Edge(6,7,8)

    # Visualize the initial graph
    g.plot_graph("Initial Graph with Capacities", s=0, t=9)

    # Calculate maximum flow from source (0) to sink (9)
    max_flow = g.max_Flow(0, 9)

    # Visualize each augmenting path
    for i in range(len(g.augmenting_paths)):
        g.plot_graph(f"Augmenting Path {i+1}", s=0, t=9, augmenting_paths=g.augmenting_paths, path_flows=g.path_flows, path_index=i)

    # Visualize the final graph with flows
    g.plot_graph("Final Graph with Capacities and Flows", s=0, t=9, show_flow=True, max_flow=max_flow)

    # Find and visualize the min-cut edges
    min_cut_edges = g.min_Cut(0)
    g.plot_graph("Min-Cut Edges", s=0, t=9, min_cut_edges=min_cut_edges)
