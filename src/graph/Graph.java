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
    public native void updatePositions();
    public native int[] getCommunitites();
    public native float[][] getClusterColors();
    public native EdgeInterm[] getEdges();
    public native Vertex[] getPositions();
    public native void startsProgram(String filename);
    public native void freeAllocatedMemory();


    public static final int WIDTH = 1500;
    public static final int HEIGHT = 800;

    private static String filename;
    private Timeline timeline;


    @Override
    public void start(Stage primaryStage) {
        Pane root = new Pane();

        // Appeler startsProgram avant d'utiliser les données natives
        startsProgram(filename);

        // float[][] color = getClusterColors();
        // int[] communitites = getCommunitites();

        // Récupérer les sommets
        List<Vertex> vertices = List.of(getPositions());

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
