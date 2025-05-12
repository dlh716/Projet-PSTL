package graph;

/**
 * Représente une arête reçue par un appel JNI depuis le programme C, en attendant de la convertir en arête du graphe
 * @see Edge
 */
@SuppressWarnings("unused")
public class EdgeC {

    private final int start;
    private final int end;
    private final double weight;


    /**
     * Crée une arête à partir de ses deux sommets et de son poids
     * @param id1 : identifiant du sommet de départ de l'arête
     * @param id2 : identifiant du sommet d'arrivée de l'arête
     * @param weight : poids de l'arête
     */
    public EdgeC(int id1, int id2, double weight) {
        this.start = id1;
        this.end = id2;
        this.weight = weight;
    }


    /**
     * @return l'identifiant du sommet de départ de l'arête
     */
    public int getStart() {
        return start;
    }
    /**
     * @return l'identifiant du sommet d'arrivée de l'arête
     */
    public int getEnd() {
        return end;
    }
    /**
     * @return le poids de l'arête
     */
    public double getWeight() {
        return weight;
    }

}