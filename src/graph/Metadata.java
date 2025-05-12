package graph;

/**
 * Représente les métadonnées du graphe retournées par le code C
 */
@SuppressWarnings("unused")
public class Metadata {
    
    private final double edge_threshold;
	private final double anti_threshold;

	public Metadata(int number_nodes, double edge_threshold, double anti_threshold, double mean_similitude) {
        this.edge_threshold = edge_threshold;
        this.anti_threshold = anti_threshold;
    }

    public Metadata(
        int number_nodes, 
        double edge_threshold, 
        double anti_threshold, 
        int number_edges, 
        int number_antiedges,
        int number_clusters) {
        this(number_nodes, edge_threshold, anti_threshold, 0.);
	}

    public double getEdgeThreshold() {
        return edge_threshold;
    }
    public double getAntiThreshold() {
        return anti_threshold;
    }
}
