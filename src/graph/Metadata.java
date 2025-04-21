package graph;

public class Metadata {
    
    public double edge_threshold, anti_threshold, mean_similitude;
    public int number_nodes, number_edges, number_antiedges, number_clusters;

    public Metadata(int number_nodes, double edge_threshold, double anti_threshold, double mean_similitude) {
        this.edge_threshold = edge_threshold;
        this.anti_threshold = anti_threshold;
        this.number_nodes = number_nodes;
        this.mean_similitude = mean_similitude;

        // not yet computed
        this.number_edges = 0;
        this.number_antiedges = 0;
        this.number_clusters = 0;
    }

    public Metadata(
        int number_nodes, 
        double edge_threshold, 
        double anti_threshold, 
        int number_edges, 
        int number_antiedges,
        int number_clusters) {
        this(number_nodes, edge_threshold, anti_threshold, 0.);

        this.number_edges = number_edges;
        this.number_antiedges = number_antiedges;
        this.number_clusters = number_clusters;
    }

    public double getEdgeThreshold() {
        return edge_threshold;
    }
    public double getAntiThreshold() {
        return anti_threshold;
    }
}
