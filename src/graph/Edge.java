package graph;

/**
 * Représente une arête du graphe
 */
public class Edge {

    public static double initial_edge_weight = 1;

    private final Vertex start, end;

    private final float color_r;
    private final float color_g;
    private final float color_b;

    private double weight;

    private boolean isDeleted = false;


    /**
     * Crée une arête à partir de ses deux sommets
     * @param start : sommet de départ de l'arête
     * @param end : sommet d'arrivée de l'arête
     * @param weight : poids de l'arête
     */
    public Edge(Vertex start, Vertex end, double weight) {
        start.addEdge(this);
        end.addEdge(this);

        this.start = start;
        this.end = end;

        Community startCommunity = start.getCommunity();
        Community endCommunity = end.getCommunity();

        if (startCommunity != null && endCommunity != null) {
            color_r = (startCommunity.getR() + endCommunity.getR()) / 2;
            color_g = (startCommunity.getG() + endCommunity.getG()) / 2;
            color_b = (startCommunity.getB() + endCommunity.getB()) / 2;
        } else {
            color_r = 0;
            color_g = 0;
            color_b = 0;
        }

        this.weight = initial_edge_weight * weight;
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
     * @return l'épaisseur de l'arête
     */
    public double getWeight() {
        return weight;
    }

    /**
     * Marque l'arête comme supprimée
     */
    public void delete() {
        isDeleted = true;
        weight = 0;
    }
    /**
     * @return true si l'arête a été supprimée, false sinon
     */
    public boolean isDeleted() {
        return isDeleted;
    }


    /**
     * @return la composante rouge de la couleur de l'arête
     */
    public float getR() {
        return color_r;
    }

    /**
     * @return la composante verte de la couleur de l'arête
     */
    public float getG() {
        return color_g;
    }

    /**
     * @return la composante bleue de la couleur de l'arête
     */
    public float getB() {
        return color_b;
    }


    /**
     * @return une représentation textuelle de l'arête (pour le débogage)
     */
    public String toString() {
        return start + " -> " + end;
    }

}