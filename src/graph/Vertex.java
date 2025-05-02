package graph;

import java.util.ArrayList;

/**
 * Représente un sommet du graphe
 */
public class Vertex {

    public static int upscale = 8;
    public static double initial_node_size = 1;
    public static double degree_scale_factor = 0;

    private int id;
    private double x, y, diameter;
    private boolean isDeleted = false;
    private final ArrayList<Edge> edges = new ArrayList<>();
    private Community community;


    /**
     * Crée un sommet à partir de ses coordonnées
     * @param x : abscisse du centre du sommet
     * @param y : ordonnée du centre du sommet
     */
    public Vertex(double x, double y) {
    	this.x = x * upscale;
    	this.y = y * upscale;
    	this.diameter = initial_node_size;
    }


    /**
     * @return identifiant du sommet
     */
    public int getId() {
        return id;
    }
    /**
     * Modifie l'identifiant du sommet
     * @param id : identifiant du sommet
     */
    public void setId(int id) {
        this.id = id;
    }

    /**
     * @return la coordonnée x du centre du sommet dans le repère modifié (JavaFX)
     */
    public double getX() {
        return x;
    }
    /**
     * @return la coordonnée y du centre du sommet dans le repère modifié (JavaFX)
     */
    public double getY() {
        return y;
    }

    /**
     * Modifie la position du sommet
     * @param x : nouvelle coordonnée x du centre du sommet
     * @param y : nouvelle coordonnée y du centre du sommet
     */
    public void updatePosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Marque le sommet comme supprimé
     */
    public void delete() {
        isDeleted = true;
        diameter = 0;
    }
    /**
     * @return true si le sommet a été supprimé, false sinon
     */
    public boolean isDeleted() {
        return isDeleted;
    }

    /**
     * @return communauté à laquelle appartient le sommet
     */
    public Community getCommunity() {
        return community;
    }
    /**
     * Modifie la communauté à laquelle appartient le sommet
     * @param c : communauté du sommet
     */
    public void setCommunity(Community c) {
        community = c;
    }

    /**
     * @return le diamètre du sommet
     */
    public double getDiameter() {
        return diameter;
    }

    /**
     * Actualise le diamètre du sommet en fonction de son degré
     */
    public void updateDiameter() {
        if (degree_scale_factor < 0)
            degree_scale_factor = 0;
        diameter = initial_node_size + degree_scale_factor * getDegree();
    }

    /**
     * @return liste des arêtes rattachées au sommet
     */
    public ArrayList<Edge> getEdges() {
        return edges;
    }
    /**
     * Ajoute une arête à la liste des arêtes rattachées au sommet
     * @param edge : arête à ajouter
     */
    public void addEdge(Edge edge) {
        edges.add(edge);
    }

    /**
     * @return degré du sommet
     */
    public int getDegree() {
        return edges.size();
    }

    /**
     * @return représentation textuelle du sommet (pour le débogage)
     */
    public String toString() {
        return id + " (" + x + ", " + y + ")";
    }

}