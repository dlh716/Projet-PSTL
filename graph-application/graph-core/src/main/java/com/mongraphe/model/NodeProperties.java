package com.mongraphe.model;

import java.util.HashMap;
import java.util.Map;

/**
 * Classe représentant les propriétés d'un nœud.
 */
public class NodeProperties {

    private final Map<String, Object> attributes = new HashMap<>();
    private double x;  // Position x
    private double y;  // Position y

    public NodeProperties() {
        // Initialisation des attributs par défaut
        this.x = 0.0;
        this.y = 0.0;
    }

    /**
     * Ajoute un attribut au nœud.
     * @param key Le nom de l'attribut.
     * @param value La valeur de l'attribut.
     */
    public void setAttribute(String key, Object value) {
        attributes.put(key, value);
    }

    /**
     * Récupère un attribut du nœud.
     * @param key Le nom de l'attribut.
     * @return La valeur de l'attribut.
     */
    public Object getAttribute(String key) {
        return attributes.get(key);
    }

    /**
     * Récupère la position X du nœud.
     * @return La position X.
     */
    public double getX() {
        return x;
    }

    /**
     * Récupère la position Y du nœud.
     * @return La position Y.
     */
    public double getY() {
        return y;
    }

    /**
     * Définit la position X du nœud.
     * @param x La position X.
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Définit la position Y du nœud.
     * @param y La position Y.
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Récupère tous les attributs du nœud.
     * @return Les attributs sous forme de Map.
     */
    public Map<String, Object> getAttributes() {
        return attributes;
    }
}
