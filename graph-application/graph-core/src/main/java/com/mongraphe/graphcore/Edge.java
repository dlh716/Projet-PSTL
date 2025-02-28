package com.mongraphe.graphcore;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Classe représentant une arête entre deux nœuds avec des attributs dynamiques.
 */
public class Edge {
    private final Node source;
    private final Node target;
    private final Map<String, Object> attributes;

    public Edge(Node source, Node target) {
        this.source = source;
        this.target = target;
        this.attributes = new HashMap<>();
    }

    public Node getSource() {
        return source;
    }

    public Node getTarget() {
        return target;
    }

    /**
     * Ajoute ou met à jour un attribut.
     * @param key Nom de l'attribut.
     * @param value Valeur de l'attribut.
     */
    public void setAttribute(String key, Object value) {
        attributes.put(key, value);
    }

    /**
     * Récupère la valeur d'un attribut.
     * @param key Nom de l'attribut.
     * @return Valeur de l'attribut ou null si non présent.
     */
    public Object getAttribute(String key) {
        return attributes.get(key);
    }

    /**
     * Retourne tous les attributs de l'arête.
     * @return Map des attributs.
     */
    public Map<String, Object> getAttributes() {
        return attributes;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Edge edge = (Edge) obj;
        return Objects.equals(source, edge.source) && Objects.equals(target, edge.target);
    }

    @Override
    public int hashCode() {
        return Objects.hash(source, target);
    }

    @Override
    public String toString() {
        return "Edge{source=" + source.getId() + ", target=" + target.getId() + ", attributes=" + attributes + "}";
    }
}

