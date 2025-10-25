/*
 * Dijkstra's Shortest Path Algorithm
 * ----------------------------------
 * Finds the shortest distance from a source vertex to all other vertices
 * in a weighted graph using a PriorityQueue (Min-Heap).
 *
 * Time Complexity: O(E log V)
 * Space Complexity: O(V)
 *
 * Example:
 * Input Graph:
 *  (0)---4-->(1)
 *   |         |
 *   1         3
 *   |         |
 *  (2)<-------(3)
 *
 * Output:
 * Shortest distances from node 0:
 * 0 -> 0 : 0
 * 0 -> 1 : 4
 * 0 -> 2 : 1
 * 0 -> 3 : 7
 */

import java.util.*;

class DijkstraAlgorithm {

    static class Edge {
        int dest, weight;
        Edge(int dest, int weight) {
            this.dest = dest;
            this.weight = weight;
        }
    }

    static class Node implements Comparable<Node> {
        int vertex, distance;
        Node(int vertex, int distance) {
            this.vertex = vertex;
            this.distance = distance;
        }
        public int compareTo(Node other) {
            return this.distance - other.distance;
        }
    }

    private static void dijkstra(List<List<Edge>> graph, int source) {
        int V = graph.size();
        int[] dist = new int[V];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[source] = 0;

        PriorityQueue<Node> pq = new PriorityQueue<>();
        pq.add(new Node(source, 0));

        while (!pq.isEmpty()) {
            Node curr = pq.poll();
            int u = curr.vertex;

            for (Edge edge : graph.get(u)) {
                int v = edge.dest;
                int weight = edge.weight;
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.add(new Node(v, dist[v]));
                }
            }
        }

        System.out.println("Shortest distances from node " + source + ":");
        for (int i = 0; i < V; i++)
            System.out.println(source + " -> " + i + " : " + dist[i]);
    }

    public static void main(String[] args) {
        int V = 4;
        List<List<Edge>> graph = new ArrayList<>();

        for (int i = 0; i < V; i++)
            graph.add(new ArrayList<>());

        graph.get(0).add(new Edge(1, 4));
        graph.get(0).add(new Edge(2, 1));
        graph.get(2).add(new Edge(3, 6));
        graph.get1().add(new Edge(3, 3));

        dijkstra(graph, 0);
    }
}
