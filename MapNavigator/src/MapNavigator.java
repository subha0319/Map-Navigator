import java.util.*;

class Graph {
    private int V; // Number of vertices
    private LinkedList<Edge>[] adjList; // Adjacency list

    // Edge class to represent an edge in the graph
    static class Edge {
        int dest;
        int weight;

        Edge(int dest, int weight) {
            this.dest = dest;
            this.weight = weight;
        }
    }

    Graph(int v) {
        V = v;
        adjList = new LinkedList[v];
        for (int i = 0; i < v; ++i) {
            adjList[i] = new LinkedList<>();
        }
    }

    void addEdge(int src, int dest, int weight) {
        adjList[src].add(new Edge(dest, weight));
        adjList[dest].add(new Edge(src, weight)); // For undirected graph
    }

    void dijkstra(int src, int dest) {
        PriorityQueue<Edge> pq = new PriorityQueue<>(Comparator.comparingInt(e -> e.weight));
        int[] dist = new int[V]; // The output array. dist[i] will hold the shortest distance from src to i

        // Initialize distances to all vertices as infinite and distance to source as 0
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[src] = 0;
        pq.add(new Edge(src, 0));

        // Parent array to store shortest path tree
        int[] parent = new int[V];
        parent[src] = -1;

        while (!pq.isEmpty()) {
            int u = pq.poll().dest;

            // Traverse through all adjacent vertices of u
            for (Edge edge : adjList[u]) {
                int v = edge.dest;
                int weight = edge.weight;

                // If there is a shorter path to v through u
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.add(new Edge(v, dist[v]));
                    parent[v] = u;
                }
            }
        }

        printPath(parent, dest);
        System.out.println("Shortest path distance: " + dist[dest]);
    }

    void printPath(int[] parent, int j) {
        if (parent[j] == -1) {
            System.out.print(j + " ");
            return;
        }
        printPath(parent, parent[j]);
        System.out.print(j + " ");
    }

    public static void main(String[] args) {
        int V = 5;
        Graph graph = new Graph(V);

        // Add edges
        graph.addEdge(0, 1, 10);
        graph.addEdge(0, 4, 5);
        graph.addEdge(1, 2, 1);
        graph.addEdge(1, 4, 2);
        graph.addEdge(2, 3, 4);
        graph.addEdge(3, 0, 7);
        graph.addEdge(3, 2, 6);
        graph.addEdge(4, 1, 3);
        graph.addEdge(4, 2, 9);
        graph.addEdge(4, 3, 2);

        int src = 0; // Source
        int dest = 3; // Destination

        graph.dijkstra(src, dest);
    }
}
