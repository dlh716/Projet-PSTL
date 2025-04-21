package graph;

/**
 * Représente une communauté du graphe
 */
public class Community {

    private final int id;
    private final float color_r;
    private final float color_g;
    private final float color_b;


    /**
     * Crée une communauté à partir de son identifiant et de ses composantes de couleur
     * @param id : identifiant de la communauté
     * @param r : composante rouge de la couleur de la communauté
     * @param g : composante verte de la couleur de la communauté
     * @param b : composante bleue de la couleur de la communauté
     */
    public Community(int id, float r, float g, float b) {
        this.id = id;
        color_r = r;
        color_g = g;
        color_b = b;
    }


    /**
     * @return l'identifiant de la communauté
     */
    public int getId() {
        return id;
    }

    /**
     * @return la composante rouge de la couleur de la communauté
     */
    public float getR() {
        return color_r;
    }

    /**
     * @return la composante verte de la couleur de la communauté
     */
    public float getG() {
        return color_g;
    }

    /**
     * @return la composante bleue de la couleur de la communauté
     */
    public float getB() {
        return color_b;
    }


    /**
     * @return une représentation textuelle de la communauté
     */
    public String toString() {
        return "Communauté " + id + " (R : " + String.format("%.2f", color_r) + ", G : " + String.format("%.2f", color_g) + ", B : " + String.format("%.2f", color_b) + ")";
    }

}
