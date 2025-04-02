package graph;

import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.sql.Time;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import com.jogamp.newt.opengl.GLWindow;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.awt.GLJPanel;
import com.jogamp.opengl.util.FPSAnimator;

import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.property.*;
import javafx.embed.swing.SwingNode;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import javafx.util.Duration;

/**
 * Classe principale pour l'affichage du graphe avec JavaFX et JOGL
 */
public class Graph extends Application implements GLEventListener, GraphSettings {

    static {
        String libnative = System.getProperty("user.dir") + "/out/libnative.so";
        System.load(libnative);
    }

    // Méthodes JNI
    public native double[][] startsProgram(String filename);
    public native Metadata initiliazeGraph(int modeCommunity, double threshold, double anti_threshold);
    public native boolean updatePositions();
    public native Vertex[] getPositions();
    public native void setNodePosition(int index, double x, double y);
    public native EdgeC[] getEdges();
    public native int[] getCommunities();
    public native float[][] getClusterColors();
    public native Metadata computeThreshold(int modeSimilitude);

    // A modifier quand le graphe est en pause
    /** Set the frequence at which the clusters are updated
     */
    public native void setSaut(int saut);
    /** Threshold indicating when the graph should be stopped
     * (if movement is less than threshold and it has been long enough then the graph stops moving)
     */
    public native void setThresholdS(double thresholdS);
    /** Set new friction *
     */
    public native void setFriction(double friction);

    // Modifiable pendant l'execution (avant prochain updatePositions)
    /** sets the repulsion mode to be used by updatePositions
     * @param mode : repulsion by degree (0), repulsion by edges (1), repulsion by communities (2)
      */
    public native void setModeRepulsion(int mode);
    /** sets force of anti-edges
     */
    public native void setAntiRepulsion(double antiedge_repulsion);
    /** sets attraction force
     */
    public native void setAttractionCoeff(double attraction_coeff);
    /** sets attraction threshold
     */
    public native void setThresholdA(double thresholdA);
    /** sets new repulsion threshold
     */
    public native void setSeuilRep(double seuilrep);
    /** the calculation depends on how big the window is
     * @param width positive real number
     * @param height positive real number
     */
    public native void setDimension(double width, double height);
    /** Set amortissement, factor dictating how the friction evolves after each update of the graph
    */
    public native void setAmortissement(double amortissement);

    public native void SetNumberClusters(int new_number_of_clusters);

    public native void freeAllocatedMemory();




    // Variables graphiques
    private GLJPanel glPanel; // JPanel pour le rendu OpenGL
    private FPSAnimator animator; // Animation du rendu OpenGL

    // Variables liées au graphe
    private List<Vertex> vertices;
    private List<Edge> edges;

    private float bg_color_r = 1.0f;
    private float bg_color_g = 1.0f;
    private float bg_color_b = 1.0f;

    public static int WIDTH = 1500; // Largeur de la fenêtre
    public static int HEIGHT = 800; // Hauteur de la fenêtre

    // Propriétés pour les différents modes du graphe
    public static final BooleanProperty isRunMode = new SimpleBooleanProperty(true);
    public static final BooleanProperty isSelectionMode = new SimpleBooleanProperty(false);
    public static final BooleanProperty isMoveMode = new SimpleBooleanProperty(false);
    public static final BooleanProperty isDeleteMode = new SimpleBooleanProperty(false);

    // Propriété pour le degré minimum des sommets
    public static final IntegerProperty minimumDegree = new SimpleIntegerProperty(0);

    // Propriété pour la fréquence de mise à jour du graphe
    public static final DoubleProperty updateFrequency = new SimpleDoubleProperty(1.0);

    // Variables pour le déplacement des sommets et de la vue
    private boolean isDraggingVertex = false;
    private Vertex draggedVertex = null;
    private double dragStartX = 0;
    private double dragStartY = 0;
    private double viewOffsetX = 0;
    private double viewOffsetY = 0;
    private boolean isDraggingGraph = false;
    private double zoomFactor = 1.0;
    private final double zoomSensitivity = 0.1;

    private Metadata init_metadata;
    private Metadata metadata;
    private Pane root;
    private Scene scene;
    private Timeline timeline;
    private GLWindow glWindow;




    public static void main(String[] args) {
        launch(args);
    }




    // -------------------------------------------------------------------------
    // Méthodes nécessaires à l'affichage du graphe
    // -------------------------------------------------------------------------

    /**
     * Méthode principale de l'application
     */
    @Override
    public void start(Stage primaryStage) {

        // Ajouter les listeners pour les différents modes du graphe
        isRunMode.addListener((obs, oldValue, newValue) -> {
            if (newValue) {
                animator.resume();
                System.out.println("Reprise de l'animation");
            } else {
                animator.pause();
                System.out.println("Pause de l'animation");
            }
        });


        // Initialisation (provisoire, devra être appelé par l'interface graphique)
        testInit();


        // Récupérer les sommets
        vertices = List.of(getPositions());

        // Récupérer les couleurs des clusters
        float[][] color = getClusterColors();

        // Récupérer les communautés de chaque sommet
        int[] communititesInterm = getCommunities();
        HashMap<Integer, Community> communities = new HashMap<>();

        for (int i = 0; i < vertices.size(); i++) {
            int community_id = communititesInterm[i];
            if (!communities.containsKey(community_id)) {
                // Créer une nouvelle communauté
                communities.put(community_id, new Community(community_id, color[i][0], color[i][1], color[i][2]));
            }
            vertices.get(i).setId(i); // Attribution d'un identifiant unique à chaque sommet
            vertices.get(i).setCommunity(communities.get(community_id)); // Attribution de la communauté à chaque sommet
        }

        // Debug : afficher les communautés
        System.out.println("\nCommunautés (" + communities.size() + ") :");
        for (Community c : communities.values())
            System.out.println("- " + c);
        System.out.println();

        // Récupérer les arêtes
        edges = new ArrayList<>();
        EdgeC[] edgesC = getEdges();
        for (EdgeC edgeC : edgesC) {
            Edge e = new Edge(vertices.get(edgeC.getStart()), vertices.get(edgeC.getEnd()), edgeC.getWeight());
            edges.add(e);
        }

        // Ajuster les rayons des sommets selon leur degré
        for (Vertex v : vertices)
            v.updateDiameter();


        // Initialisation de OpenGL avec JOGL
        GLProfile glProfile = GLProfile.get(GLProfile.GL2);
        GLCapabilities capabilities = new GLCapabilities(glProfile);

        // Utiliser GLJPanel au lieu de GLWindow pour une meilleure intégration avec JavaFX
        glPanel = new GLJPanel(capabilities);
        glPanel.addGLEventListener(this);

        // Ajouter les listeners pour la souris
        addMouseListeners();

        // Créer un JPanel contenant le glPanel
        JPanel mainPanel = new JPanel();
        mainPanel.setLayout(new java.awt.BorderLayout());
        mainPanel.add(glPanel, java.awt.BorderLayout.CENTER);
        mainPanel.setPreferredSize(new java.awt.Dimension(WIDTH, HEIGHT));

        // Créer un JFrame pour contenir le panel
        JFrame frame = new JFrame();
        frame.getContentPane().add(mainPanel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Créer un SwingNode pour intégrer le JPanel dans JavaFX
        SwingNode swingNode = new SwingNode();

        // Exécuter sur le thread EDT de Swing
        SwingUtilities.invokeLater(() -> {
            swingNode.setContent(mainPanel);
        });

        // Créer la scène JavaFX avec le SwingNode
        Pane root = new Pane();
        root.getChildren().add(swingNode);
        Scene scene = new Scene(root, WIDTH, HEIGHT);

        // Démarrer l'animation
        animator = new FPSAnimator(glPanel, 60);
        animator.start();




        // Tests d'actions sur le graphe (en attendant l'interface graphique)
        testActions();




        primaryStage.setTitle("Graphique avec JOGL et JavaFX");
        primaryStage.setScene(scene);
        //primaryStage.setMaximized(true);
        primaryStage.setOnCloseRequest(event -> Platform.exit());
        primaryStage.show();
    }


    /**
     * Trouve le sommet à la position (x, y)
     * @param x Position x
     * @param y Position y
     * @return le sommet trouvé, ou null s'il n'y en a pas
     */
    private Vertex findVertexAt(double x, double y) {
        for (Vertex v : vertices) {
            double dx = x - v.getX();
            double dy = y - v.getY();
            double distance = Math.sqrt(dx * dx + dy * dy);
            if (distance <= (v.getDiameter()/2) + 5) // Ajouter une marge pour faciliter la sélection
                return v;
        }
        return null;
    }

    /**
     * Ajoute les listeners pour la souris
     */
    private void addMouseListeners() {

        glPanel.addMouseListener(new MouseListener() {

            // Gère le maintien du clic de la souris
            @Override
            public void mousePressed(MouseEvent e) {
                double x = e.getX() - WIDTH / 2.0 + viewOffsetX;
                double y = HEIGHT / 2.0 - e.getY() + viewOffsetY;

                // Vérifier si un sommet est cliqué
                draggedVertex = findVertexAt(x, y);

                // Déplacer un sommet
                if (isSelectionMode.get() && draggedVertex != null)
                    isDraggingVertex = true;

                // Se déplacer dans le graphe
                if (isMoveMode.get())
                    isDraggingGraph = true;
            }

            // Gère le relâchement du clic de la souris
            @Override
            public void mouseReleased(MouseEvent e) {
                if (isSelectionMode.get() && isDraggingVertex && draggedVertex != null) {
                    isDraggingVertex = false;
                    setNodePosition(draggedVertex.getId(), draggedVertex.getX(), draggedVertex.getY());
                    draggedVertex = null;
                }
                isDraggingGraph = false;
            }

            @Override
            public void mouseClicked(MouseEvent e) {}

            @Override
            public void mouseEntered(MouseEvent e) {}

            @Override
            public void mouseExited(MouseEvent e) {}
        });

        glPanel.addMouseMotionListener(new MouseMotionListener() {

            // Gère le déplacement de la souris
            @Override
            public void mouseDragged(MouseEvent e) {

                // Déplacer un sommet
                if (isSelectionMode.get() && isDraggingVertex && draggedVertex != null) {
                    double newX = e.getX() - WIDTH / 2.0 + viewOffsetX;
                    double newY = HEIGHT / 2.0 - e.getY() + viewOffsetY;
                    draggedVertex.updatePosition(newX, newY);
                }

                // TODO
                // Se déplacer dans le graphe
                if (isMoveMode.get() && isDraggingGraph) {
                    double deltaX = e.getX() - dragStartX;
                    double deltaY = e.getY() - dragStartY;
                    viewOffsetX -= deltaX;
                    viewOffsetY += deltaY;
                    dragStartX = e.getX();
                    dragStartY = e.getY();
                }
                glPanel.repaint();
            }

            @Override
            public void mouseMoved(MouseEvent e) {}
        });

        glPanel.addMouseWheelListener(e -> {
            // TODO
            /*int notches = e.getWheelRotation();
            if (notches < 0) {
                // Zoom avant
                zoomFactor += zoomSensitivity;
            } else {
                // Zoom arrière
                zoomFactor -= zoomSensitivity;
                if (zoomFactor < 0.1) {
                    zoomFactor = 0.1;
                }
            }

            // Calculer le facteur de décalage de la vue pour un zoom centré
            double zoomCenterX = e.getX() - WIDTH / 2.0; // Position du centre du zoom sur l'écran
            double zoomCenterY = HEIGHT / 2.0 - e.getY(); // Position du centre du zoom sur l'écran

            // Ajuster la position de la vue en fonction du zoom
            viewOffsetX += (zoomCenterX * zoomSensitivity);
            viewOffsetY -= (zoomCenterY * zoomSensitivity);
            glPanel.repaint();*/
        });
    }


    /**
     * Initialise OpenGL
     * @param drawable Objet OpenGL
     */
    @Override
    public void init(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2(); // Utiliser GL2 au lieu de GL
        gl.glClearColor(bg_color_r, bg_color_g, bg_color_b, 1.0f); // Couleur de fond de l'écran
        gl.glEnable(GL2.GL_DEPTH_TEST); // Activer le test de profondeur pour les objets 3D

        // Activer le rendu de points circulaires
        // gl.glEnable(GL2.GL_BLEND);
        // gl.glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE_MINUS_SRC_ALPHA);
        gl.glEnable(GL2.GL_POINT_SMOOTH);
        gl.glHint(GL2.GL_POINT_SMOOTH_HINT, GL2.GL_FASTEST);
        // gl.glHint(GL2.GL_POINT_SMOOTH_HINT, GL2.GL_NICEST);
    }

    /**
     * Libère les ressources OpenGL
     * @param drawable Objet OpenGL
     */
    @Override
    public void dispose(GLAutoDrawable drawable) {
    }

    /**
     * Affiche le graphe avec OpenGL
     * @param drawable Objet OpenGL
     */
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2(); // Utiliser GL2 au lieu de GL

        gl.glClear(GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT); // Effacer l'écran
        gl.glLoadIdentity();

        // Appliquer la translation de la vue (déplacement de la souris)
        gl.glTranslated(-viewOffsetX, -viewOffsetY, 0);

        // Mettre à jour les positions des points si le mode de simulation est RUN
        if (isRunMode.get()) {

            boolean is_running = updatePositions();
            List<Vertex> updatedVertices = List.of(getPositions());
            for (int i = 0; i < updatedVertices.size(); i++) {

                Vertex v = vertices.get(i);

                // Mise à jour des coordonnées des sommets
                v.updatePosition(updatedVertices.get(i).getX(), updatedVertices.get(i).getY());

                // Mise à jour du degré minimum des sommets
                /*int min_degree = minimumDegree.get();
                if (v.getDegree() >= min_degree && !root.getChildren().contains(v)) {
                    root.getChildren().add(v);
                    for (Edge e : v.getEdges())
                        if (!root.getChildren().contains(e) && e.getStart().getDegree() >= min_degree && e.getEnd().getDegree() >= min_degree)
                            root.getChildren().add(e);
                } else if (v.getDegree() < minimumDegree.get()) {
                    root.getChildren().remove(v);
                    for (Edge e : v.getEdges())
                        root.getChildren().remove(e);
                }*/
            }
        }

        // Récupérer les nouvelles données
        long startTime = System.nanoTime();

        // Dessiner les sommets
        for (Vertex v : vertices) {
            if (v.getDegree() < minimumDegree.get())
                continue;

            gl.glPointSize((float)v.getDiameter()); // Taille du sommet

            gl.glBegin(GL2.GL_POINTS);
            Community c = v.getCommunity();
            if (c != null)
                gl.glColor3f(c.getR(), c.getG(), c.getB()); // Couleur de la communauté
            else
                gl.glColor3f(0, 0, 0);
            gl.glVertex2d(v.getX(), v.getY()); // Position du sommet
            gl.glEnd();
        }

        // Dessiner les arêtes
        for (Edge e : edges) {
            if (e.getStart() == null || e.getEnd() == null)
                continue;
            if (e.getStart().getDegree() < minimumDegree.get() || e.getEnd().getDegree() < minimumDegree.get())
                continue;
            gl.glLineWidth((float)e.getWeight());  // Epaisseur de l'arête
            gl.glBegin(GL2.GL_LINES);
            gl.glColor3f(e.getR(), e.getG(), e.getB());  // Couleur de l'arête
            gl.glVertex2d(e.getStart().getX(), e.getStart().getY());  // Position du début de l'arête
            gl.glVertex2d(e.getEnd().getX(), e.getEnd().getY());  // Position de la fin de l'arête
            gl.glEnd();
        }

        long endTime = System.nanoTime();
        double elapsedTimeMs = (endTime - startTime) / 1_000_000.0;
        //System.out.println("Temps d'affichage : " + elapsedTimeMs + " ms");
    }

    @Override
    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glMatrixMode(GL2.GL_PROJECTION);
        gl.glLoadIdentity();

        // Définir une projection avec le centre de l'écran à (0,0)
        double left = -WIDTH / 2.0;
        double right = WIDTH / 2.0;
        double bottom = -HEIGHT / 2.0;
        double top = HEIGHT / 2.0;

        gl.glOrtho(left, right, bottom, top, -1, 1);
        gl.glMatrixMode(GL2.GL_MODELVIEW);
    }

    @Override
    public void stop() {
        if (animator != null) {
            animator.stop();
        }
        freeAllocatedMemory();
    }




    // -------------------------------------------------------------------------
    // Exemples d'initialisation et d'actions sur le graphe
    // -------------------------------------------------------------------------

    /**
     * Exemple d'initialisation du graphe (à remplacer par l'interface graphique)
     */
    private void testInit() {
        // Initialisation du graphe avec le fichier à charger, la méthode de similitude et la méthode de détection de communautés
        String sample1 = "samples/iris.csv";
        String sample2 = "samples/predicancerNUadd9239.csv";
        initGraph(sample2, GraphData.SimilitudeMode.CORRELATION, GraphData.NodeCommunity.LOUVAIN);

        setScreenSize(WIDTH, HEIGHT); // Taille de l'écran du graphe
        setBackgroundColor(0.0f, 0.0f, 0.0f); // Couleur de fond du graphe
        setUpscale(5); // Facteur d'agrandissement pour le graphe
        setInitialNodeSize(3); // Taille initiale d'un sommet
        setDegreeScaleFactor(0.3); // Facteur d'agrandissement selon le degré d'un sommet
    }




    /**
     * Exemple d'actions sur le graphe (en attendant l'interface graphique)
     */
    private void testActions() {

        Timeline tl1 = new Timeline(
                new KeyFrame(Duration.seconds(15), e -> {
                    setMode(GraphData.GraphMode.SELECTION);
                    System.out.print("Switch to " + getMode());
                    System.out.println(" - Vous pouvez vous déplacer dans le graphe");
                })
        );
        tl1.setCycleCount(1);
        tl1.play();

        Timeline tl2 = new Timeline(
                new KeyFrame(Duration.seconds(25), e -> {
                    setMode(GraphData.GraphMode.RUN);
                    System.out.print("Switch to " + getMode());
                    System.out.println(" - Vous pouvez sélectionner et déplacer des sommets");
                })
        );
        tl2.setCycleCount(1);
        tl2.play();

        Timeline tl2b = new Timeline(
                new KeyFrame(Duration.seconds(30), e -> {
                    setMode(GraphData.GraphMode.SELECTION);
                    System.out.print("Switch to " + getMode());
                    System.out.println(" - Vous pouvez vous déplacer dans le graphe");
                })
        );
        tl2b.setCycleCount(1);
        tl2b.play();

        Timeline tl3 = new Timeline(
                new KeyFrame(Duration.seconds(40), e -> {
                    setMode(GraphData.GraphMode.RUN);
                    System.out.print("Back to " + getMode());
                    System.out.println(" - Exécution du graphe (en mouvement)");
                })
        );
        tl3.setCycleCount(1);
        tl3.play();

        Timeline tl4 = new Timeline(
                new KeyFrame(Duration.seconds(50), e -> {
                    setMiniumDegree(10);
                    System.out.println("Remove nodes with degree < 10");
                })
        );
        tl4.setCycleCount(1);
        tl4.play();

        Timeline tl5 = new Timeline(
                new KeyFrame(Duration.seconds(55), e -> {
                    setMiniumDegree(0);
                    System.out.println("Reset minimum degree restriction");
                })
        );
        tl5.setCycleCount(1);
        tl5.play();

    }




    // -------------------------------------------------------------------------
    // Initialisation
    // -------------------------------------------------------------------------

    /**
     * Initialise le graphe avec les données du fichier .csv
     * @param path Chemin du fichier .csv à charger
     * @param mode Mode de similitude à utiliser
     * @param community Mode de détection de communautés à utiliser
     * @return les données du fichier .csv
     * @see GraphData.SimilitudeMode
     * @see GraphData.NodeCommunity
     */
    @Override
    public double[][] initGraph(String path, GraphData.SimilitudeMode mode, GraphData.NodeCommunity community) {
        if (path == null || path.isEmpty())
            throw new RuntimeException("initGraph : Chemin du fichier non spécifié.");

        // Appeler startsProgram avant d'utiliser les données natives
        double[][] data = startsProgram(path);

        // Déterminer le mode de similitude à utiliser
        if (mode == null)
            throw new RuntimeException("initGraph : Mode de similitude non spécifié.");
        int modeSimilitude = getModeSimilitude(mode);

        init_metadata = computeThreshold(modeSimilitude);
        if (init_metadata == null)
            throw new RuntimeException("initGraph : Une erreur est survenue lors du calcul des seuils.");

        double recommendedThreshold = init_metadata.getEdgeThreshold();
        double recommendedAntiThreshold = init_metadata.getAntiThreshold();

        System.out.println("Seuil recommandé pour les arêtes : " + recommendedThreshold);
        System.out.println("Seuil recommandé pour les anti-arêtes : " + recommendedAntiThreshold);

        // Valeurs imposées pour le moment (à modifier)
        recommendedThreshold = 0.966;
        recommendedAntiThreshold = 0.6;

        // Déterminer le mode de détection de communautés à utiliser
        if (community == null)
            throw new RuntimeException("initGraph : Mode de détection de communautés non spécifié.");
        int modeCommunity = getModeCommunity(community);

        metadata = initiliazeGraph(modeCommunity, recommendedThreshold, recommendedAntiThreshold);

        return data;
    }

    /**
     * Initialise la taille de l'écran du graphe
     * @param width  Largeur de l'écran (en px)
     * @param height Hauteur de l'écran (en px)
     */
    @Override
    public void setScreenSize(int width, int height) {
        if (width <= 0 || height <= 0)
            throw new RuntimeException("setScreenSize : Taille de l'écran (" + width + "x" + height + ") non valide.");
        WIDTH = width;
        HEIGHT = height;
        //setDimension(WIDTH, HEIGHT); // TODO
    }

    /**
     * @param upscale Facteur d'agrandissement pour le graphe
     */
    @Override
    public void setUpscale(int upscale) {
        if (upscale < 0)
            throw new RuntimeException("setUpscale : Facteur d'agrandissement (" + upscale + ") non valide.");
        Vertex.upscale = upscale;
    }

    /**
     * @param size Taille initiale d'un sommet
     */
    @Override
    public void setInitialNodeSize(double size) {
        if (size <= 0)
            throw new RuntimeException("setInitialNodeSize : Taille initiale d'un sommet (" + size + ") non valide.");
        Vertex.initial_node_size = size;
    }

    /**
     * @param factor Facteur d'agrandissement selon le degré d'un sommet (0 pour que la taille soit identique pour tous les sommets, > 0  pour faire varier la taille proportionnellement au degré)
     */
    @Override
    public void setDegreeScaleFactor(double factor) {
        if (factor < 0)
            throw new RuntimeException("setDegreeScaleFactor : Facteur d'agrandissement selon le degré (" + factor + ") non valide.");
        Vertex.degree_scale_factor = factor;
    }




    // -------------------------------------------------------------------------
    // Mode de sélection
    // -------------------------------------------------------------------------

    /**
     * @return le mode actuel du graphe
     * @see GraphData.GraphMode
     */
    @Override
    public GraphData.GraphMode getMode() {
        if (isRunMode.get()) {
            return GraphData.GraphMode.RUN;
        } else if (isSelectionMode.get()) {
            return GraphData.GraphMode.SELECTION;
        } else if (isMoveMode.get()) {
            return GraphData.GraphMode.MOVE;
        } else if (isDeleteMode.get()) {
            return GraphData.GraphMode.DELETE;
        }
        return null;
    }

    /**
     * Définit le mode du graphe
     * @param mode le mode à définir
     * @see GraphData.GraphMode
     */
    @Override
    public void setMode(GraphData.GraphMode mode) {
        if (mode == null)
            throw new RuntimeException("setMode : Mode non spécifié.");
        isRunMode.set(mode == GraphData.GraphMode.RUN);
        isSelectionMode.set(mode == GraphData.GraphMode.SELECTION);
        isMoveMode.set(mode == GraphData.GraphMode.MOVE);
        isDeleteMode.set(mode == GraphData.GraphMode.DELETE);
    }




    // -------------------------------------------------------------------------
    // Paramètres de la simulation
    // -------------------------------------------------------------------------

    /**
     * @return le seuil recommandé pour les arêtes
     */
    @Override
    public double getRecommendedThreshold() {
        if (init_metadata == null)
            throw new RuntimeException("getRecommendedThreshold : Métadonnées non initialisées. Veuillez appeler initGraph() avant.");
        return init_metadata.getEdgeThreshold();
    }

    /**
     * @return le seuil recommandé pour les anti-arêtes
     */
    @Override
    public double getRecommendedAntiThreshold() {
        if (init_metadata == null)
            throw new RuntimeException("getRecommendedAntiThreshold : Métadonnées non initialisées. Veuillez appeler initGraph() avant.");
        return init_metadata.getAntiThreshold();
    }

    /**
     * @return le seuil actuel pour les arêtes
     */
    @Override
    public double getThreshold() {
        if (init_metadata == null)
            throw new RuntimeException("getThreshold : Métadonnées non initialisées. Veuillez appeler initGraph() avant.");
        return metadata.getEdgeThreshold();
    }

    /**
     * @return le seuil actuel pour les anti-arêtes
     */
    @Override
    public double getAntiThreshold() {
        if (metadata == null)
            throw new RuntimeException("getAntiThreshold : Métadonnées non initialisées. Veuillez appeler initGraph() avant.");
        return metadata.getAntiThreshold();
    }

    /**
     * Change le seuil pour les arêtes
     * @param threshold Nouveau seuil pour les arêtes
     */
    @Override
    public void setThreshold(double threshold) {
        // TODO
    }

    /**
     * Change le seuil pour les anti-arêtes
     * @param antiThreshold Nouveau seuil pour les anti-arêtes
     */
    @Override
    public void setAntiThreshold(double antiThreshold) {
        // TODO
    }

    /**
     * Affiche les sommets dont le degré est supérieur ou égal à degree
     * @param degree Degré minimum des sommets à afficher
     */
    @Override
    public void setMiniumDegree(int degree) {
        if (degree < 0)
            throw new RuntimeException("setMiniumDegree : Degré minimum (" + degree + ") non valide.");
        minimumDegree.set(degree);
    }

    /**
     * Modifie la couleur de fond du graphe
     * @param color_r Composante rouge de la couleur
     * @param color_g Composante verte de la couleur
     * @param color_b Composante bleue de la couleur
     */
    @Override
    public void setBackgroundColor(float color_r, float color_g, float color_b) {
        if (color_r < 0 || color_r > 1 || color_g < 0 || color_g > 1 || color_b < 0 || color_b > 1)
            throw new RuntimeException("setBackgroundColor : Couleur (" + color_r + ", " + color_g + ", " + color_b + ") non valide.");
        this.bg_color_r = color_r;
        this.bg_color_g = color_g;
        this.bg_color_b = color_b;
    }




    // -------------------------------------------------------------------------
    // Options supplémentaires
    // -------------------------------------------------------------------------

    /**
     * Exporte le graphe en image PNG
     * @param path Chemin de l'image PNG à exporter
     */
    @Override
    public void exportToPng(String path) {
        /*
    private void drawCircle(GL2 gl, double x, double y, double radius, int segments) {
        gl.glBegin(GL2.GL_TRIANGLE_FAN);
        gl.glVertex2d(x, y); // Center point
        for (int i = 0; i <= segments; i++) {
            double angle = 2.0 * Math.PI * i / segments;
            gl.glVertex2d(x + radius * Math.cos(angle), y + radius * Math.sin(angle));
        }
        gl.glEnd();
    }
     */
        // TODO
    }




    // -------------------------------------------------------------------------
    // Outils
    // -------------------------------------------------------------------------

    /**
     * @param mode Mode de similitude
     * @return l'identifiant du mode de similitude
     */
    private static int getModeSimilitude(GraphData.SimilitudeMode mode) {
        int modeSimilitude = -1;
        switch (mode) {
            case GraphData.SimilitudeMode.CORRELATION -> modeSimilitude = 0;
            case GraphData.SimilitudeMode.DISTANCE_COSINE -> modeSimilitude = 1;
            case GraphData.SimilitudeMode.DISTANCE_EUCLIDIENNE -> modeSimilitude = 2;
            case GraphData.SimilitudeMode.NORME_L1 -> modeSimilitude = 3;
            case GraphData.SimilitudeMode.NORME_LINF -> modeSimilitude = 4;
            case GraphData.SimilitudeMode.KL_DIVERGENCE -> modeSimilitude = 5;
        }
        if (modeSimilitude == -1)
            throw new RuntimeException("Mode de similitude non reconnu.");
        return modeSimilitude;
    }

    /**
     * @param community Mode de détection de communautés
     * @return l'identifiant du mode de détection de communautés
     */
    private static int getModeCommunity(GraphData.NodeCommunity community) {
        int modeCommunity = -1;
        switch (community) {
            case GraphData.NodeCommunity.LOUVAIN -> modeCommunity = 0;
            case GraphData.NodeCommunity.LOUVAIN_PAR_COMPOSANTE -> modeCommunity = 1;
            case GraphData.NodeCommunity.LEIDEN -> modeCommunity = 2;
            case GraphData.NodeCommunity.LEIDEN_CPM -> modeCommunity = 3;
            case GraphData.NodeCommunity.COULEURS_SPECIALES -> modeCommunity = 4;
        }
        if (modeCommunity == -1)
            throw new RuntimeException("Mode de détection de communautés non reconnu.");
        return modeCommunity;
    }

}