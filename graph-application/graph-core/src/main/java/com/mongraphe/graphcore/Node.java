package com.mongraphe.graphcore;

import com.mongraphe.model.NodeProperties;

public class Node {

    private final String id;
    private final NodeProperties properties;

    public Node(String id) {
        this.id = id;
        this.properties = new NodeProperties();
    }

    /**
     * Obtient l'ID du nœud.
     * @return L'ID du nœud.
     */
    public String getId() {
        return id;
    }

    /**
     * Obtient les propriétés du nœud.
     * @return Les propriétés du nœud.
     */
    public NodeProperties getProperties() {
        return properties;
    }

    /**
     * Définit un attribut pour le nœud.
     * @param key Le nom de l'attribut.
     * @param value La valeur de l'attribut.
     */
    public void setAttribute(String key, Object value) {
        properties.setAttribute(key, value);
    }

    /**
     * Récupère un attribut du nœud.
     * @param key Le nom de l'attribut.
     * @return La valeur de l'attribut.
     */
    public Object getAttribute(String key) {
        return properties.getAttribute(key);
    }

    /**
     * Définit la position X du nœud.
     * @param x La position X.
     */
    public void setX(double x) {
        properties.setX(x);
    }

    /**
     * Définit la position Y du nœud.
     * @param y La position Y.
     */
    public void setY(double y) {
        properties.setY(y);
    }

    /**
     * Récupère la position X du nœud.
     * @return La position X.
     */
    public double getX() {
        return properties.getX();
    }

    /**
     * Récupère la position Y du nœud.
     * @return La position Y.
     */
    public double getY() {
        return properties.getY();
    }
}
