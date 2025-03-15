package graph;

import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import javafx.util.Duration;

import java.util.List;

import static graph.Vertex.resetCount;

public class Graph extends Application {

    static {
        String libnative = System.getProperty("user.dir") + "/out/libnative.so";
        System.load(libnative);
    }


    // Méthodes natives
    /* Updates the graph
     * @return boolean indicating whether the graph reached a stable state
     */
    public native boolean updatePositions();
    public native int[] getCommunitites();
    public native float[][] getClusterColors();
    public native EdgeInterm[] getEdges();
    public native Vertex[] getPositions();

    /** read file with name filename
     * @param filename : name of csv file to process
     * @return double[][] array containing data of csv file
     */
    public native double[][] startsProgram(String filename);

    /* computes recommended threshold
     * @param modeSimilitude : Correlation (0), Distance Cosine (1), Distance Euclidienne (2), Norme L1 (3), Norme Linf (4), KL divergence (5)
     * @return Metadata containing the recommended thresholds and the number of nodes in the graph
     */
    public native Metadata computeThreshold(int modeSimilitude);

    /** Initialize the graph according 
     * @param modeCommunity : Louvain (0), Louvain par composante (1) ou Leiden (2) ou Leiden CPM (3) ou couleurs speciales (4)
     * @param threshold : threshold used to compute edges
     * @param anti_threshold : threshold used to compute anti_edges
     */
    public native Metadata initiliazeGraph(int modeCommunity, double threshold, double anti_threshold);
    
    /************************ A modifier quand le graphe est en pause * ************************************/
    /** Set the frequence at which the clusters are updated 
     * @param saut
     */
    public native void setSaut(int saut);
    /** Threshold indicating when the graph should be stopped 
     * (if movement is less than threshold and it has been long enough then the graph stops moving)
     * @param thresholdS
     */
    public native void setThresholdS(double thresholdS);

    /** Set new friction *
     * @param friction
    **/
    public native void setFriction(double friction);
    /****************************************************************************************************/

    /********************* Modifiable pendant l'execution (avant prochain updatePositions) *****************/
    /** sets the repulsion mode to be used by updatePositions
     * @param mode : repulsion by degree (0), repulsion by edges (1), repulsion by communities (2)
      */
    public native void setModeRepulsion(int mode);
    /** sets force of anti-edges
     * @param antiedge_repulsion
     */
    public native void setAntiRepulsion(double antiedge_repulsion);
    /** sets attraction force
     * @param attraction_coeff
     */
    public native void setAttractionCoeff(double attraction_coeff);
    /** sets attraction threshold
     * @param thresholdA
     */
    public native void setThresholdA(double thresholdA);
    /** sets new repulsion threshold
     * @param seuilrep
     */
    public native void setSeuilRep(double seuilrep);

    /** the calculation depends on how big the window is
     * @param width positive real number
     * @param height positive real number
     */
    public native void setDimension(double width, double height);

    /** Set amortissement, factor dictating how the friction evolves after each update of the graph
     * @param amortissement
    */
    public native void setAmortissement(double amortissement);

    /** Set node positions
     * @param index id of points
     * @param x first coordinate of the point
     * @param y second coordinate of the point
     */
    public native void setNodePosition(int index, double x, double y);
    /****************************************************************************************************/
    
    
    public native void freeAllocatedMemory();

    public static final int WIDTH = 1500;
    public static final int HEIGHT = 800;

    private static String filename;
    private Timeline timeline;


    @Override
    public void start(Stage primaryStage) {
        Pane root = new Pane();

        // Appeler startsProgram avant d'utiliser les données natives
        // C'est à l'utilisateur de choisir le second argument
        double[][] data  = startsProgram(filename);

        Metadata metadata = computeThreshold(0);

        // C'est à l'utilisateur de choisir les arguments
        metadata = initiliazeGraph(0, metadata.edge_threshold, metadata.anti_threshold);
        // float[][] color = getClusterColors();
        // int[] communitites = getCommunitites();

        // Récupérer les sommets
        List<Vertex> vertices = List.of(getPositions());
        System.out.println("LENGTH: " + getPositions().length);
        // Récupérer les arêtes
        EdgeInterm[] edgesInterm = getEdges();
        for (int i = 0; i < edgesInterm.length; i++) {
            Edge e = new Edge(vertices.get(edgesInterm[i].getStart()), vertices.get(edgesInterm[i].getEnd()));
            root.getChildren().add(e);
        }

        // Ajouter les sommets
        root.getChildren().addAll(vertices);

        // Création de la Timeline pour mettre à jour le graphe
        timeline = new Timeline();
        KeyFrame keyFrame = new KeyFrame(Duration.seconds(0.05), event -> {
            // Met à jour les positions du graphe
            updatePositions();

            // Récupère les nouvelles positions et les met à jour

            // Efface les anciennes positions (les sommets et les arêtes)
            root.getChildren().clear();
            resetCount();

            // Ajoute les nouveaux sommets
            List<Vertex> updatedVertices = List.of(getPositions());

            // Récupérer les arêtes
            EdgeInterm[] updatedEdgesInterm = getEdges();
            for (int i = 0; i < updatedEdgesInterm.length; i++) {
                Edge e = new Edge(updatedVertices.get(updatedEdgesInterm[i].getStart()), updatedVertices.get(updatedEdgesInterm[i].getEnd()));
                root.getChildren().add(e);
            }

            root.getChildren().addAll(updatedVertices);
        });

        // Définir la fréquence de mise à jour (ici toutes les 0.05 secondes)
        timeline.getKeyFrames().add(keyFrame);
        timeline.setCycleCount(Timeline.INDEFINITE);
        timeline.play();

        // Créer un timer pour arrêter la timeline après 5 secondes
        javafx.animation.Timeline stopTimer = new javafx.animation.Timeline(
                new KeyFrame(Duration.seconds(5), e -> {
                    timeline.stop();
                    System.out.println("Timeline arrêtée après 5 secondes");
                })
        );
        stopTimer.setCycleCount(1);
        stopTimer.play();

        primaryStage.setTitle("Graphe interactif");
        primaryStage.setScene(new Scene(root, WIDTH, HEIGHT));
        //primaryStage.setMaximized(true);
        primaryStage.setOnCloseRequest(event -> Platform.exit());
        primaryStage.show();
    }

    public static void main(String[] args) {
        if (args.length > 0) {
            filename = args[0];
        } else {
            System.out.println("Aucun argument fourni.");
        }

        launch(args);
    }
}
