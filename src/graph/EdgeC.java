package graph;

/**
 * Représente une arête reçue par un appel JNI depuis le programme C, en attendant de la convertir en arête JavaFX
 * @see Edge
 */
public class EdgeC {

    private final int start;
    private final int end;
    private final double weight;


    /**
     * Crée une arête à partir de l'id de ses deux sommets
     * @param id_start : sommet de départ de l'arête
     * @param id_end : sommet d'arrivée de l'arête
     * @param weight : poids de l'arête
     */
    public EdgeC(int id_start, int id_end, double weight) {
        this.start = id_start;
        this.end = id_end;
        this.weight = weight;
    }


    /**
     * @return l'id du sommet de départ de l'arête
     */
    public int getStart() {
        return start;
    }

    /**
     * @return l'id du sommet d'arrivée de l'arête
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
