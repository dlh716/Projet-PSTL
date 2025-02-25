package backendinterface;

public class Edge {
    private int node1;
    private int node2;
    private double weight;

    public Edge(int id1, int id2, double weight)
    {
        this.node1 = id1;
        this.node2 = id2;
        this.weight = weight;
    }

}
