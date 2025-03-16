package graph;

import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;

import java.util.ArrayList;

import static graph.Graph.HEIGHT;
import static graph.Graph.WIDTH;

/**
 * Représente un sommet du graphe JavaFX
 */
public class Vertex extends Circle {

    public static int upscale = 1;
    public static double initial_node_size = 1;
    public static double degree_scale_factor = 0;

    private int id;
    private final ArrayList<Edge> edges = new ArrayList<>();
    private Community community;
    private Graph graph;


    /**
     * Crée un sommet à partir de ses coordonnées
     * @param x : abscisse du centre du sommet
     * @param y : ordonnée du centre du sommet
     */
    public Vertex(double x, double y) {
        super(convertX(x), convertY(y), 5, Color.BLUE);
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
        return getCenterX();
    }
    /**
     * @return la coordonnée y du centre du sommet dans le repère modifié (JavaFX)
     */
    public double getY() {
        return getCenterY();
    }

    /**
     * @return la coordonnée x du centre du sommet dans le repère initial (C)
     */
    public double getRedoX() {
        return redoX(getCenterX());
    }
    /**
     * @return la coordonnée y du centre du sommet dans le repère initial (C)
     */
    public double getRedoY() {
        return redoY(getCenterY());
    }


    /**
     * Modifie la communauté à laquelle appartient le sommet
     * @param c : communauté du sommet
     */
    public void setCommunity(Community c) {
        community = c;
        changeColor(c.getR(), c.getG(), c.getB());
    }

    /**
     * @return communauté à laquelle appartient le sommet
     */
    public Community getCommunity() {
        return community;
    }


    /**
     * Modifie la couleur du sommet
     * @param r : composante rouge de la couleur du sommet
     * @param g : composante verte de la couleur du sommet
     * @param b : composante bleue de la couleur du sommet
     */
    public void changeColor(double r, double g, double b) {
        setFill(Color.color(r, g, b));
    }


    /**
     * Modifie le graphe auquel appartient le sommet
     * @param g : graphe auquel appartient le sommet
     */
    public void setGraph(Graph g) {
        graph = g;
        initializeEventHandlers();
    }
    private void initializeEventHandlers() {
        setOnMousePressed(e -> {
            if (Graph.isSelectionMode.get()) {
                System.out.println("Sommet sélectionné : " + this);
            }
        });

        setOnMouseDragged(e -> {
            if (Graph.isSelectionMode.get() && graph != null) {
                double newX = e.getX();
                double newY = e.getY();
                setCenterX(e.getX());
                setCenterY(e.getY());
                graph.setNodePosition(id, redoX(newX), redoY(newY));
                edges.forEach(edge -> edge.update(this));
            }
        });
    }


    /**
     * Actualise le rayon du sommet en fonction de son degré
     */
    public void updateRadius() {
        if (degree_scale_factor < 0)
            degree_scale_factor = 0;
        setRadius(initial_node_size + degree_scale_factor * getDegree());
    }

    /**
     * Modifie la position du sommet
     * @param x : nouvelle coordonnée x du centre du sommet
     * @param y : nouvelle coordonnée y du centre du sommet
     */
    public void updatePosition(double x, double y) {
        setCenterX(x);
        setCenterY(y);
        edges.forEach(edge -> edge.update(this));
    }


    /**
     * Ajoute une arête à la liste des arêtes rattachées au sommet
     * @param edge : arête à ajouter
     */
    public void addEdge(Edge edge) {
        edges.add(edge);
    }

    /**
     * @return liste des arêtes rattachées au sommet
     */
    public ArrayList<Edge> getEdges() {
        return edges;
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
        return id + " (" + getCenterX() + ", " + getCenterY() + ")";
    }


    private static double convertX(double x) {
        if (upscale <= 0)
            upscale = 1;
        return x * upscale + (WIDTH/2);
    }
    private static double convertY(double y) {
        if (upscale <= 0)
            upscale = 1;
        return y * upscale + (HEIGHT/2);
    }
    private static double redoX(double x) {
        if (upscale <= 0)
            upscale = 1;
        return (x - (WIDTH/2)) / upscale;
    }
    private static double redoY(double y) {
        if (upscale <= 0)
            upscale = 1;
        return (y - (HEIGHT/2)) / upscale;
    }

}
