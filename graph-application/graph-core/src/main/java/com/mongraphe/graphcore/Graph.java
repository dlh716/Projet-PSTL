package com.mongraphe.graphcore;

import java.util.List;
import java.util.Set;

/**
 * Interface représentant un graphe avec des nœuds, des arêtes et des attributs dynamiques.
 */
public interface Graph {

    /**
     * Ajoute un nœud au graphe.
     * @param node Le nœud à ajouter.
     */
    void addNode(Node node);

    /**
     * Supprime un nœud du graphe.
     * @param node Le nœud à supprimer.
     */
    void removeNode(Node node);

    /**
     * Ajoute une arête entre deux nœuds.
     * @param source Nœud source.
     * @param target Nœud cible.
     * @return L'arête créée.
     */
    Edge addEdge(Node source, Node target);

    /**
     * Supprime une arête entre deux nœuds.
     * @param source Nœud source.
     * @param target Nœud cible.
     */
    void removeEdge(Node source, Node target);

    /**
     * Vérifie si un nœud est présent dans le graphe.
     * @param node Le nœud à vérifier.
     * @return true si le nœud est présent, false sinon.
     */
    boolean containsNode(Node node);

    /**
     * Vérifie si une arête existe entre deux nœuds.
     * @param source Nœud source.
     * @param target Nœud cible.
     * @return true si l'arête existe, false sinon.
     */
    boolean containsEdge(Node source, Node target);

    /**
     * Récupère tous les nœuds du graphe.
     * @return Un ensemble contenant tous les nœuds.
     */
    Set<Node> getNodes();

    /**
     * Récupère toutes les arêtes du graphe.
     * @return Une liste d'arêtes.
     */
    List<Edge> getEdges();

    /**
     * Retourne les voisins d'un nœud donné.
     * @param node Le nœud dont on veut les voisins.
     * @return Une liste de nœuds voisins.
     */
    List<Node> getNeighbors(Node node);

    /**
     * Efface le contenu du graphe (réinitialisation).
     */
    void clear();
}
