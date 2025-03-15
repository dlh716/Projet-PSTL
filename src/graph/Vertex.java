package graph;

import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;

import java.util.ArrayList;

import static graph.Graph.HEIGHT;
import static graph.Graph.WIDTH;

public class Vertex extends Circle {

    public static int upscale = 1;
    public static double initial_node_size = 1;
    public static double degree_scale_factor = 0;

    private int id;
    private final ArrayList<Edge> edges = new ArrayList<>();
    private Community community;
    private Graph graph;


    public Vertex(double x, double y) {
        super(convertX(x), convertY(y), 5, Color.BLUE);
    }


    public void setId(int id) {
        this.id = id;
    }

    public double getX() {
        return getCenterX();
    }
    public double getY() {
        return getCenterY();
    }

    public void setCommunity(Community c) {
        community = c;
        changeColor(c.getR(), c.getG(), c.getB());
    }
    public Community getCommunity() {
        return community;
    }

    public void changeColor(double r, double g, double b) {
        setFill(Color.color(r, g, b));
    }

    public void setGraph(Graph g) {
        graph = g;
        initializeEventHandlers();
    }
    private void initializeEventHandlers() {
        setOnMousePressed(e -> {
            if (Graph.isSelectionMode.get()) {
                System.out.println("Point sélectionné : " + this);
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

    public void updateRadius() {
        if (degree_scale_factor < 0)
            degree_scale_factor = 0;
        setRadius(initial_node_size + degree_scale_factor * getDegree());
    }
    public void updatePosition(double x, double y) {
        setCenterX(x);
        setCenterY(y);
        edges.forEach(edge -> edge.update(this));
    }

    public void addEdge(Edge edge) {
        edges.add(edge);
    }
    public ArrayList<Edge> getEdges() {
        return edges;
    }
    public int getDegree() {
        return edges.size();
    }

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
