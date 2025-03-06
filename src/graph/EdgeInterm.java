package graph;

public class EdgeInterm {
    private int start;
    private int end;
    private double weight;

    public EdgeInterm(int id1, int id2, double weight) {
        this.start = id1;
        this.end = id2;
        this.weight = weight;
    }

    public int getStart() {
        return start;
    }
    public int getEnd() {
        return end;
    }
    public double getWeight() {
        return weight;
    }
}
