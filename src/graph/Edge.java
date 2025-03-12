package graph;

import javafx.scene.paint.Color;
import javafx.scene.shape.Line;

public class Edge extends Line {
    private final Vertex start, end;

    public Edge(Vertex start, Vertex end) {
        super(start.getX(), start.getY(), end.getX(), end.getY());
        start.addEdge(this);
        end.addEdge(this);

        this.start = start;
        this.end = end;
        setStroke(Color.BLACK);
        setStrokeWidth(0.5);
    }

    public void update(Vertex vertex) {
        if (vertex == start) {
            setStartX(vertex.getX());
            setStartY(vertex.getY());
        } else if (vertex == end) {
            setEndX(vertex.getX());
            setEndY(vertex.getY());
        }
        //System.out.println("Mise à jour de l'arrête " + this);
    }

    public String toString() {
        return start + " -> " + end;
    }

}
