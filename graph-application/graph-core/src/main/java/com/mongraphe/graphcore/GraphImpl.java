package com.mongraphe.graphcore;

import java.util.List;
import java.util.Set;

import java.util.*;

/**
 * Implémentation d'un graphe avec des nœuds et des arêtes ayant des attributs dynamiques.
 */
public class GraphImpl implements Graph {

    private final Set<Node> nodes;
    private final Set<Edge> edges;
    private final Map<Node, List<Edge>> adjacencyList;

    public GraphImpl() {
        this.nodes = new HashSet<>();
        this.edges = new HashSet<>();
        this.adjacencyList = new HashMap<>();
    }

    @Override
    public void addNode(Node node) {
        if (nodes.add(node)) {
            adjacencyList.putIfAbsent(node, new ArrayList<>());
        }
    }

    @Override
    public void removeNode(Node node) {
        if (nodes.remove(node)) {
            List<Edge> toRemove = adjacencyList.remove(node);
            if (toRemove != null) {
                edges.removeAll(toRemove);
            }
            adjacencyList.values().forEach(edgeList -> edgeList.removeIf(edge -> edge.getSource().equals(node) || edge.getTarget().equals(node)));
        }
    }

    @Override
    public Edge addEdge(Node source, Node target) {
        if (nodes.contains(source) && nodes.contains(target)) {
            Edge edge = new Edge(source, target);
            if (edges.add(edge)) {
                adjacencyList.get(source).add(edge);
                adjacencyList.get(target).add(edge); // Si graphe non orienté
            }
            return edge;
        }
        return null;
    }

    @Override
    public void removeEdge(Node source, Node target) {
        edges.removeIf(edge -> edge.getSource().equals(source) && edge.getTarget().equals(target));
        adjacencyList.getOrDefault(source, Collections.emptyList()).removeIf(edge -> edge.getTarget().equals(target));
        adjacencyList.getOrDefault(target, Collections.emptyList()).removeIf(edge -> edge.getSource().equals(source));
    }

    @Override
    public boolean containsNode(Node node) {
        return nodes.contains(node);
    }

    @Override
    public boolean containsEdge(Node source, Node target) {
        return edges.stream().anyMatch(edge -> edge.getSource().equals(source) && edge.getTarget().equals(target));
    }

    @Override
    public Set<Node> getNodes() {
        return Collections.unmodifiableSet(nodes);
    }

    @Override
    public List<Edge> getEdges() {
        return new ArrayList<>(edges);
    }

    @Override
    public List<Node> getNeighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();
        for (Edge edge : adjacencyList.getOrDefault(node, Collections.emptyList())) {
            if (edge.getSource().equals(node)) {
                neighbors.add(edge.getTarget());
            } else {
                neighbors.add(edge.getSource());
            }
        }
        return neighbors;
    }

    @Override
    public void clear() {
        nodes.clear();
        edges.clear();
        adjacencyList.clear();
    }
}
