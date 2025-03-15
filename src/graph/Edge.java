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

        Community startCommunity = start.getCommunity();
        Community endCommunity = end.getCommunity();

        if (startCommunity != null && endCommunity != null) {
            if (startCommunity.getId() == endCommunity.getId()) {
                setStroke(Color.color(startCommunity.getR(), startCommunity.getG(), startCommunity.getB()));
            } else {
                setStroke(Color.color((startCommunity.getR() + endCommunity.getR()) / 2, (startCommunity.getG() + endCommunity.getG()) / 2, (startCommunity.getB() + endCommunity.getB()) / 2));
            }
        } else {
            setStroke(Color.BLACK);
        }

        setStrokeWidth(0.3);
    }


    public Vertex getStart() {
        return start;
    }
    public Vertex getEnd() {
        return end;
    }


    public void update(Vertex vertex) {
        if (vertex == start) {
            setStartX(vertex.getX());
            setStartY(vertex.getY());
        } else if (vertex == end) {
            setEndX(vertex.getX());
            setEndY(vertex.getY());
        }
    }


    public String toString() {
        return start + " -> " + end;
    }

}
