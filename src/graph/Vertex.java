package graph;

import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;

import java.util.ArrayList;

import static graph.Graph.HEIGHT;
import static graph.Graph.WIDTH;

public class Vertex extends Circle {
    public static int count = 0;

    protected double x, y;
    private final int id = ++count;

    private static final int ratio = 15;

    protected final ArrayList<Edge> edges = new ArrayList<>();

    public Vertex(double x, double y, double radius, Color color) {
        super(x*ratio + ((double)WIDTH/2), y*ratio + ((double)HEIGHT/2), radius, color);

        setOnMousePressed(e -> {
            this.x = e.getX() - getCenterX();
            this.y = e.getY() - getCenterY();
            System.out.println("Point sélectionné : " + this);
        });

        setOnMouseDragged(e -> {
            setCenterX(e.getX() - this.x);
            setCenterY(e.getY() - this.y);
            edges.forEach(edge -> edge.update(this));
        });
    }
    public Vertex(double x, double y) {
        this(x, y, 5, Color.BLUE);
    }

    public void update(double x, double y) {
        setCenterX(x);
        setCenterY(y);
        edges.forEach(edge -> edge.update(this));
    }

    public double getX() {
        return getCenterX();
    }
    public double getY() {
        return getCenterY();
    }

    public void addEdge(Edge edge) {
        edges.add(edge);
    }

    public String toString() {
        return id + " (" + getCenterX() + ", " + getCenterY() + ")";
    }

    public static void resetCount() {
        count = 0;
    }
}
