package graph;

import javafx.scene.paint.Color;
import javafx.scene.shape.Line;

/**
 * Représente une arête du graphe JavaFX
 */
public class Edge extends Line {

    public static double initial_edge_weight = 0.3;

    private final Vertex start, end;


    /**
     * Crée une arête à partir de ses deux sommets
     * @param start : sommet de départ de l'arête
     * @param end : sommet d'arrivée de l'arête
     * @param weight : poids de l'arête
     */
    public Edge(Vertex start, Vertex end, double weight) {

        super(start.getX(), start.getY(), end.getX(), end.getY());
        start.addEdge(this);
        end.addEdge(this);

        this.start = start;
        this.end = end;

        Community startCommunity = start.getCommunity();
        Community endCommunity = end.getCommunity();

        if (startCommunity != null && endCommunity != null) {
            if (startCommunity.getId() == endCommunity.getId())
                setStroke(Color.color(startCommunity.getR(), startCommunity.getG(), startCommunity.getB()));
            else
                setStroke(Color.color((startCommunity.getR() + endCommunity.getR()) / 2, (startCommunity.getG() + endCommunity.getG()) / 2, (startCommunity.getB() + endCommunity.getB()) / 2));
        } else {
            setStroke(Color.BLACK);
        }

        setStrokeWidth(initial_edge_weight * weight);
    }


    /**
     * @return le sommet de départ de l'arête
     */
    public Vertex getStart() {
        return start;
    }
    /**
     * @return le sommet d'arrivée de l'arête
     */
    public Vertex getEnd() {
        return end;
    }


    /**
     * Met à jour les coordonnées de l'arête si l'un de ses sommets est déplacé
     * @param vertex : sommet à mettre à jour
     */
    public void update(Vertex vertex) {
        if (vertex == start) {
            setStartX(vertex.getX());
            setStartY(vertex.getY());
        } else if (vertex == end) {
            setEndX(vertex.getX());
            setEndY(vertex.getY());
        }
    }


    /**
     * @return une représentation textuelle de l'arête (pour le débogage)
     */
    public String toString() {
        return start + " -> " + end;
    }

}
