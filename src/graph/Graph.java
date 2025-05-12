package graph;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.SwingUtilities;

import com.jogamp.newt.event.KeyEvent;
import com.jogamp.newt.event.KeyListener;
import com.jogamp.newt.event.MouseEvent;
import com.jogamp.newt.event.MouseListener;
import com.jogamp.newt.event.WindowAdapter;
import com.jogamp.newt.event.WindowEvent;
import com.jogamp.newt.javafx.NewtCanvasJFX;
import com.jogamp.newt.opengl.GLWindow;
import com.jogamp.opengl.GL4;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.util.FPSAnimator;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;

/**
 * Classe principale pour l'affichage du graphe avec JavaFX et JOGL
 */
public class Graph extends Application implements GLEventListener, GraphSettings {

    static {
        String libnative = System.getProperty("user.dir") + "/out/libnative.so";
        System.load(libnative);
    }

    // Méthodes JNI
    private native double[][] startsProgram(String filename);
	private native Metadata computeThreshold(int modeSimilitude, int edge_factor);
	private native Metadata initializeGraph(int modeCommunity, double threshold, double anti_threshold);
	private native Metadata initializeDot(String filepath, int modeCommunity);

	@SuppressWarnings("unused")
	private native void setDimension(double width, double height); // TODO (la méthode setDimension ne fonctionne pas correctement dans le code C)

	private native boolean updatePositions();
	private native Vertex[] getPositions();
	private native void setNodePosition(int index, double x, double y);
	private native EdgeC[] getEdges();
	private native int[] getCommunities();
	private native float[][] getClusterColors();
	private native void setSaut(int saut);
	private native void setThresholdS(double thresholdS);
	private native void setFriction(double friction);
	private native void setModeRepulsion(int mode);
	private native void setAntiRepulsion(double antiedge_repulsion);
	private native void setAttractionCoeff(double attraction_coeff);
	private native void setThresholdA(double thresholdA);
	private native void setSeuilRep(double seuilrep);
	private native void setAmortissement(double amortissement);
	private native void SetNumberClusters(int new_number_of_clusters);
	private native void deleteNode(int index);
	private native void setKmeansMode(boolean md);
	private native int[] getHistogram();
	private native void freeAllocatedMemory();

    // Variables graphiques
    private FPSAnimator animator; // Animation du rendu OpenGL

    // Variables liées au graphe
    private List<Vertex> vertices; // Liste des sommets du graphe
    private List<Edge> edges; // Liste des arêtes du graphe

    private float bg_color_r = 0.0f; // Composante rouge de la couleur de fond
    private float bg_color_g = 0.0f; // Composante verte de la couleur de fond
    private float bg_color_b = 0.0f; // Composante bleue de la couleur de fond

    public static int WIDTH = 1000; // Largeur de la fenêtre
    public static int HEIGHT = 1000; // Hauteur de la fenêtre
    public static int GRAPH_UPSCALE = 5; // Facteur d'agrandissment du graphe
    public static double CORRELATION_THRESHOLD = 0.5; // Définir la valeur seuil pour l'affichage de la corrélation

	public static String IMAGE_EXPORT_PATH = "capture"; // Chemin d'exportation des images

    // Propriétés pour les différents modes du graphe
    public static final BooleanProperty isRunMode = new SimpleBooleanProperty(true);
    public static final BooleanProperty isSelectionMode = new SimpleBooleanProperty(false);
    public static final BooleanProperty isMoveMode = new SimpleBooleanProperty(false);
    public static final BooleanProperty isDeleteMode = new SimpleBooleanProperty(false);

    // Propriété pour le degré minimum des sommets
    public static final IntegerProperty minimumDegree = new SimpleIntegerProperty(0);

    // Variables pour le déplacement des sommets et de la vue
    private boolean isDraggingVertex = false;
    private Vertex selectedVertex = null;
    private double dragStartX = 0;
    private double dragStartY = 0;
    private double viewOffsetX = 0;
    private double viewOffsetY = 0;
    private boolean isDraggingGraph = false;
    private double zoomFactor = 1.0;

    private Metadata init_metadata;
    private Metadata metadata;
    private GLWindow glWindow;

    // Variables pour les buffers et shaders
    private FloatBuffer projectionMatrix;
    
	private int verticesShaderProgram;
	private int vertexBuffer;
	private int vertexColorBuffer;
	private int vertexSizeBuffer;
	private int vertexVisibilityBuffer;
	private float[] vertexPoints;
	private float[] vertexSizes;
	private float[] vertexColors;
	private float[] vertexVisibility;

	private int edgesShaderProgram;
	private int edgeBuffer;
	private int edgeColorBuffer;
	private int edgeSizeBuffer;
	private int edgeVisibilityBuffer;
	private float[] edgePoints;
	private float[] edgeSizes;
	private float[] edgeColors;
	private float[] edgeVisibility;
	
	private int doubleCircleShaderProgram;
	private int bezierShaderProgram;
	
	private int bezierBuffer;
	private int bezierColorBuffer;
	private int bezierSizeBuffer;
	private int bezierVisibilityBuffer;
	private float[] bezierPoints;
	private float[] bezierSizes;
	private float[] bezierColors;
	private float[] bezierVisibility;
	
	// Variables pour le déplacement
	private double dragOffsetX = 0;
	private double dragOffsetY = 0;




    public static void main(String[] args) {
        launch(args);
    }




    // -------------------------------------------------------------------------
    // Méthodes nécessaires à l'affichage du graphe
    // -------------------------------------------------------------------------

    /**
	 * Méthode principale du module graphique
	 * @param primaryStage Fenêtre principale de l'application
	 */
    @Override
	@SuppressWarnings("unused")
    public void start(Stage primaryStage) {

        // Listeners pour le mode d'exécution RUN
    	isRunMode.addListener((obs, oldValue, newValue) ->
    	    Platform.runLater(() -> {
    	        if (newValue) {
    	            //System.out.println("Try to resume the animation...");
    	            if (!animator.isAnimating()) {
    	                animator.resume();
    	                //System.out.println("Animation resumed");
    	            }
    	        } else {
    	            //System.out.println("Try to pause the animation...");
    	            if (animator.isAnimating()) {
    	                animator.pause();
    	                //System.out.println("Animation paused");
    	            }
    	        }
    	    })
    	);


        // Initialisation (provisoire, devra être appelé par l'interface utilisateur)
        testInit();


        // Récupérer les sommets
        vertices = List.of(getPositions());
		System.out.println("Number of vertices: " + vertices.size());

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
		System.out.println("Number of communities: " + communities.size());

        // Récupérer les arêtes
        edges = new ArrayList<>();
        EdgeC[] edgesC = getEdges();
        for (EdgeC edgeC : edgesC) {
            Edge e = new Edge(vertices.get(edgeC.getStart()), vertices.get(edgeC.getEnd()), edgeC.getWeight());
            edges.add(e);
        }
		System.out.println("Number of edges: " + edges.size());

        // Ajuster les rayons des sommets selon leur degré
        for (Vertex v : vertices)
            v.updateDiameter();

		System.out.println("================================================================");
		System.out.println("Once the graph is initialized, you can use the following keys:");
		System.out.println("1: switch to RUN mode");
		System.out.println("2: switch to SELECTION mode (select and move vertices)");
		System.out.println("3: switch to MOVE mode (translate/zoom the graph)");
		System.out.println("4: switch to DELETE mode (delete vertices)");
		System.out.println("5: toggle the minimum degree of vertices (0 or 1)");
		System.out.println("6: export the graph to a PNG file");
		System.out.println("7: export the graph to an upgraded PNG file");
		System.out.println("================================================================");

		// Initialisation de OpenGL avec JOGL
        GLProfile glProfile = GLProfile.get(GLProfile.GL4);
        GLCapabilities capabilities = new GLCapabilities(glProfile);
        capabilities.setDoubleBuffered(true);
        capabilities.setHardwareAccelerated(true);

        // Créer un GLWindow (OpenGL)
        glWindow = GLWindow.create(capabilities);

        // Ajouter un GLEventListener pour dessiner le triangle
        glWindow.addGLEventListener(this);

        // Ajouter un WindowListener pour fermer proprement la fenêtre
        glWindow.addWindowListener(new WindowAdapter() {
        @Override
        public void windowDestroyed(WindowEvent e) {
        	System.exit(0); // Quitter l'application lorsque la fenêtre est fermée
        }
        });

        // Créer un NewtCanvasJFX pour intégrer OpenGL dans JavaFX
        NewtCanvasJFX newtCanvas = new NewtCanvasJFX(glWindow);
        newtCanvas.setWidth(WIDTH);
        newtCanvas.setHeight(HEIGHT);

        // Ajouter les listeners pour la souris
        addMouseListeners();

        // Ajouter les listeners pour le clavier
        addKeyListeners();

        // Créer la scène JavaFX avec le SwingNode
        Pane root = new Pane();
        root.getChildren().add(newtCanvas);
        Scene scene = new Scene(root, WIDTH, HEIGHT);
        newtCanvas.requestFocus();

        // Démarrer l'animation
        animator = new FPSAnimator(glWindow, 60);
        animator.setExclusiveContext(false);
        animator.start();
        
        primaryStage.setTitle("Graphique avec JOGL et JavaFX");
        primaryStage.setScene(scene);
        primaryStage.setOnCloseRequest(event -> Platform.exit());
        primaryStage.show();
    }

    /**
     * Trouve le sommet à la position (x, y)
     * @param x Position x déjà ajustée avec viewOffsetX
     * @param y Position y déjà ajustée avec viewOffsetY
     * @return le sommet trouvé, ou null s'il n'y en a pas
     */
    private Vertex findVertexAt(double x, double y) {
    	for (Vertex v : vertices) {
	    	if (v.isDeleted()) continue;
		
	    	double dx = x - v.getX();
	    	double dy = y - v.getY();
	    	double distance = Math.sqrt(dx * dx + dy * dy);
	    	double vertexDiameter = v.getDiameter();
	    	double margin = (vertexDiameter < 3) ? 3 : 0; // Ajouter une marge pour faciliter la sélection
	    	double selectionRadius = ((v.getDiameter() / 2) + margin) / zoomFactor; 
	
	    	if (distance <= selectionRadius)
	    		return v;
	    }
    	return null;
    }

    /**
     * Ajoute les listeners pour les actions de la souris (clic, déplacement, scroll)
     */
    private void addMouseListeners() {
        glWindow.addMouseListener(new MouseListener() {

			/**
			 * Gère le maintien du clic de la souris
			 * @param e Événement de la souris
			 */
        	@Override
        	public void mousePressed(MouseEvent e) {
	        	// Calculer les coordonnées ajustées avec le décalage de vue
        		double x = (e.getX() - WIDTH / 2.0) / zoomFactor + viewOffsetX;
        		double y = (HEIGHT / 2.0 - e.getY()) / zoomFactor + viewOffsetY;

	        	// Vérifier si un sommet est cliqué
	        	selectedVertex = findVertexAt(x, y);
	
	        	// Déplacer un sommet
	        	if (isSelectionMode.get() && selectedVertex != null) {
		        	isDraggingVertex = true;
		        	dragOffsetX = selectedVertex.getX() - x;
		        	dragOffsetY = selectedVertex.getY() - y;
	        	}
	
	        	// Se déplacer dans le graphe
	        	else if (isMoveMode.get()) {
		        	isDraggingGraph = true;
		        	dragStartX = e.getX();
		        	dragStartY = e.getY();
	        	}
        	}

			/**
			 * Gère le relâchement du clic de la souris
			 * @param e Événement de la souris
			 */
            @Override
            public void mouseReleased(MouseEvent e) {
                // Déplacer un sommet
                if (isSelectionMode.get() && isDraggingVertex && selectedVertex != null) {
                    isDraggingVertex = false;
					System.out.println("Move vertex to (" + selectedVertex.getX() + ", " + selectedVertex.getY() + ")");
                    setNodePosition(selectedVertex.getId(), selectedVertex.getX() / GRAPH_UPSCALE, selectedVertex.getY() / GRAPH_UPSCALE);
                    vertexPoints[selectedVertex.getId() * 2] = (float) selectedVertex.getX();
                    vertexPoints[selectedVertex.getId() * 2 + 1] = (float) selectedVertex.getY();
                    selectedVertex = null;
                }
                isDraggingGraph = false;
            }

			/**
			 * Gère le clic de la souris
			 * @param e Événement de la souris
			 */
            @Override
            public void mouseClicked(MouseEvent e) {
            	double x = (e.getX() - WIDTH / 2.0) / zoomFactor + viewOffsetX;
            	double y = (HEIGHT / 2.0 - e.getY()) / zoomFactor + viewOffsetY;

                // Vérifier si un sommet est cliqué
                selectedVertex = findVertexAt(x, y);

                // Obtenir les informations sur un sommet
                if (isSelectionMode.get() && selectedVertex != null) {
                    System.out.println("Selected vertex: " + selectedVertex);
                }

                // Supprimer un sommet
                else if (isDeleteMode.get() && selectedVertex != null) {
                    selectedVertex.delete();
                    System.out.println("Deleted vertex: " + selectedVertex);
                    deleteNode(selectedVertex.getId());
                    SwingUtilities.invokeLater(() -> glWindow.display());
                }
            }

            @Override
            public void mouseEntered(MouseEvent e) {}

            @Override
            public void mouseExited(MouseEvent e) {}

			/**
			 * Gère le glisser-déposer de la souris
			 * @param e Événement de la souris
			 */
            @Override
            public void mouseDragged(MouseEvent e) {
                boolean updated = false;
                // Déplacer un sommet
                if (isSelectionMode.get() && isDraggingVertex && selectedVertex != null) {
                    // Calculer les coordonnées ajustées pour le drag
                	double x = (e.getX() - WIDTH / 2.0) / zoomFactor + viewOffsetX;
                	double y = (HEIGHT / 2.0 - e.getY()) / zoomFactor + viewOffsetY;
                    
                    // Appliquer l'offset du drag
                    double newX = x + dragOffsetX;
                    double newY = y + dragOffsetY;
                    
                    selectedVertex.updatePosition(newX, newY);
                    vertexPoints[selectedVertex.getId() * 2] = (float) selectedVertex.getX();
                    vertexPoints[selectedVertex.getId() * 2 + 1] = (float) selectedVertex.getY();
                    updated = true;
                }
                
                // Se déplacer dans le graphe
                else if (isMoveMode.get() && isDraggingGraph) {
                    double deltaX = e.getX() - dragStartX;
                    double deltaY = e.getY() - dragStartY;

                    double speedFactor = 0.8 * 1/zoomFactor;

                    viewOffsetX -= deltaX * speedFactor;
                    viewOffsetY += deltaY * speedFactor;
                    dragStartX = e.getX();
                    dragStartY = e.getY();
                    updated = true;
                }
                
                // Forcer l'affichage sur le thread UI pour éviter les lags visuels pendant le drag
                if (updated) SwingUtilities.invokeLater(() -> glWindow.display());
            }

            @Override
            public void mouseMoved(MouseEvent e) {}

			/**
			 * Gère le scroll de la souris
			 * @param e Événement de la souris
			 */
            @Override
            public void mouseWheelMoved(MouseEvent e) {
                if (isMoveMode.get()) {
                    float[] rotation = e.getRotation(); // [x, y]
                    float scrollY = rotation[1];
                    double zoomAmount = 1.1;

                    if (scrollY == 0) return;

                    // Calculer les coordonnées avant zoom
                    double mouseXBefore = (e.getX() - WIDTH / 2.0) / zoomFactor + viewOffsetX;
                    double mouseYBefore = (HEIGHT / 2.0 - e.getY()) / zoomFactor + viewOffsetY;

                    if (scrollY > 0) {
                        zoomFactor *= zoomAmount;
                    } else {
                        zoomFactor /= zoomAmount;
                    }

                    // Limite du facteur de zoom pour éviter les zooms extrêmes
                    zoomFactor = Math.max(0.1, Math.min(zoomFactor, 10.0));

                    // Calculer les coordonnées après zoom
                    double mouseXAfter = (e.getX() - WIDTH / 2.0) / zoomFactor + viewOffsetX;
                    double mouseYAfter = (HEIGHT / 2.0 - e.getY()) / zoomFactor + viewOffsetY;

                    // Ajuster le décalage pour maintenir le point sous la souris
                    viewOffsetX += (mouseXBefore - mouseXAfter);
                    viewOffsetY += (mouseYBefore - mouseYAfter);

                    updateProjectionMatrix();
                    SwingUtilities.invokeLater(() -> glWindow.display());
                }
            }
        });
    }

    /**
     * Ajoute les listeners pour le clavier (touche pressée)
     */
    private void addKeyListeners() {
        glWindow.addKeyListener(new KeyListener() {

			/**
			 * Gère la pression d'une touche
			 * @param e Événement de la touche pressée
			 */
        	@Override
        	public void keyPressed(KeyEvent e) {
        	    char keyChar = e.getKeyChar(); // Récupère le caractère associé à la touche pressée
        	    
        	    // Vérifier si la touche pressée est un chiffre entre 1 et 9
        	    if (keyChar >= '1' && keyChar <= '9') {
        	        int keyNumber = keyChar - '0';  // Convertit le caractère en un nombre entier

					// Obtenir la date et l'heure actuelles pour le nom des images à sauvegarder
					LocalDateTime now = LocalDateTime.now();
					DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd_HHmmss");
					String formattedDate = now.format(formatter);

        	        // Switch en fonction de la touche pressée
        	        switch (keyNumber) {
						case 1:
							setMode(GraphData.GraphMode.RUN);
							break;
        	            case 2:
        	                setMode(GraphData.GraphMode.SELECTION);
        	                break;
						case 3:
							setMode(GraphData.GraphMode.MOVE);
							break;
        	            case 4:
        	                setMode(GraphData.GraphMode.DELETE);
        	                break;
        	            case 5:
        	            	if (getMinimumDegree() > 0) {
        	            		setMinimumDegree(0);
        	            		System.out.println("Minimum vertex degree set to 0");
        	            	} else {
        	            		setMinimumDegree(1);
        	            		System.out.println("Minimum vertex degree set to 1");
        	            	}
        	                break;
        	            case 6:
        	                exportToPng("graph_" + formattedDate + ".png");
        	                break;
        	            case 7:
							upgradedExportToPng("upgradedGraph_" + formattedDate + ".png");
        	                break;
						default:
							System.out.println("Invalid key pressed: " + keyChar);
        	        }
        	    } else {
					System.out.println("Invalid key pressed: " + keyChar);
				}

            }

            @Override
            public void keyReleased(KeyEvent e) {}
        });
    }

    /**
     * Initialise OpenGL
     * @param drawable Objet OpenGL
     */
    @Override
    public void init(GLAutoDrawable drawable) {

    	// Utiliser GL4 au lieu de GL pour accéder aux fonctionnalités OpenGL 4
    	GL4 gl = drawable.getGL().getGL4();

    	// Définir la couleur de fond de l'écran (en RGB)
    	gl.glClearColor(bg_color_r, bg_color_g, bg_color_b, 1.0f);

    	// Contrôler la taille des points
    	gl.glEnable(GL4.GL_PROGRAM_POINT_SIZE);

    	// Test de profondeur pour gérer les objets devant ou derrière d'autres objets
    	gl.glEnable(GL4.GL_DEPTH_TEST);

        // Initialisation des buffers pour les sommets et arêtes
        initializeArrays();
	    createVertexBuffers(gl);
	    createEdgeBuffers(gl);
	    createBezierBuffers(gl);
	    verticesShaderProgram = createShaderProgram(gl, POINT_VERTEX_SHADER, POINT_FRAGMENT_SHADER);
	    edgesShaderProgram = createShaderProgram(gl, EDGE_VERTEX_SHADER, EDGE_FRAGMENT_SHADER);
	    doubleCircleShaderProgram = createShaderProgram(gl, DOUBLE_CIRCLE_VERTEX_SHADER, DOUBLE_CIRCLE_FRAGMENT_SHADER);
	    bezierShaderProgram = createShaderProgram(gl, BEZIER_VERTEX_SHADER, BEZIER_FRAGMENT_SHADER);
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
	    GL4 gl = drawable.getGL().getGL4();

	    // Met à jour la matrice de transformation avec les offsets actuels
	    updateProjectionMatrix();

	    gl.glClear(GL4.GL_COLOR_BUFFER_BIT | GL4.GL_DEPTH_BUFFER_BIT);

        if (isRunMode.get()) {
            boolean is_running = updatePositions();
			if (!is_running) {
				// Si la simulation est terminée, on passe en mode sélection
				setMode(GraphData.GraphMode.SELECTION);
			} else {
				List<Vertex> updatedVertices = List.of(getPositions());
				for (int i = 0; i < updatedVertices.size(); i++) {
					Vertex v = vertices.get(i);
					// Mise à jour des coordonnées des sommets
					v.updatePosition(updatedVertices.get(i).getX(), updatedVertices.get(i).getY());
				}
			}
        }

        prepareVertexRenderData();
	    prepareEdgeRenderData();
	    renderVertices(gl);
	    renderEdges(gl);
    }

	/**
	 * Réajuster la matrice de projection en fonction des nouvelles dimensions de la fenêtre
	 * @param drawable Drawable OpenGL (GLAutoDrawable) utilisé pour dessiner sur la fenêtre
	 * @param x Coordonnée X de l'angle inférieur gauche de la nouvelle fenêtre
	 * @param y Coordonnée Y de l'angle inférieur gauche de la nouvelle fenêtre
	 * @param width Nouvelle largeur de la fenêtre en pixels
	 * @param height Nouvelle hauteur de la fenêtre en pixels
	 */
    @Override
    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
	    // Réajustement de la matrice de projection pour tenir compte de la taille de la fenêtre
	    float left = -width / 2f;
	    float right = width / 2f;
	    float bottom = -height / 2f;
	    float top = height / 2f;
	    float near = -1f;
	    float far = 1f;

	    float[] orthoMatrix = new float[] {
	        2f / (right - left), 0, 0, 0,
	        0, 2f / (top - bottom), 0, 0,
	        0, 0, -2f / (far - near), 0,
	        -(right + left) / (right - left), -(top + bottom) / (top - bottom), -(far + near) / (far - near), 1f
	    };

	    // Sauvegarder la matrice dans un buffer pour l'envoyer comme uniforme
	    this.projectionMatrix = FloatBuffer.wrap(orthoMatrix);
    }

	/**
	 * Arrête l'animation et libère la mémoire allouée
	 */
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
	 * @see GraphData.SimilitudeMode
	 * @see GraphData.NodeCommunity
     */
	@SuppressWarnings("unused")
    private void testInit() {
        String sample1 = "samples/iris.csv"; // Fichier d'exemple à 150 sommets
        String sample2 = "samples/predicancerNUadd9239.csv"; // Fichier d'exemple à 9238 sommets
		int edge_factor = 5; // Facteur d'arêtes souhaité par l'utilisateur (plus il est élevé, plus le graphe sera dense)

		// Initialisation du graphe avec les données du fichier .csv
        initGraphCsv(sample2, GraphData.SimilitudeMode.CORRELATION, GraphData.NodeCommunity.LOUVAIN, edge_factor);

        setScreenSize(WIDTH, HEIGHT); // Taille de l'écran du graphe
        setBackgroundColor(bg_color_r, bg_color_g, bg_color_r); // Couleur de fond du graphe
        setUpscale(GRAPH_UPSCALE); // Facteur d'agrandissement pour le graphe
        setInitialNodeSize(15); // Taille initiale d'un sommet
        setDegreeScaleFactor(0.15); // Facteur d'agrandissement selon le degré d'un sommet
		setThresholdS(0.01);
    }




    // -------------------------------------------------------------------------
    // Initialisation
    // -------------------------------------------------------------------------

    /**
     * Initialise le graphe avec les données du fichier .csv
     * @param path Chemin du fichier .csv à charger
     * @param mode Mode de similitude à utiliser
     * @param community Mode de détection de communautés à utiliser
	 * @param edge_factor Ratio d'arêtes souhaité par l'utilisateur (plus il est élevé, plus le graphe sera dense)
     * @return les données du fichier .csv
     * @see GraphData.SimilitudeMode
     * @see GraphData.NodeCommunity
     */
    @Override
    public double[][] initGraphCsv(String path, GraphData.SimilitudeMode mode, GraphData.NodeCommunity community, int edge_factor) {
        if (path == null || path.isEmpty())
            throw new RuntimeException("initGraphCsv : Chemin du fichier non spécifié.");

        // Appeler startsProgram avant d'utiliser les données natives
        double[][] data = startsProgram(path);

        // Déterminer le mode de similitude à utiliser
        if (mode == null)
            throw new RuntimeException("initGraphCsv : Mode de similitude non spécifié.");
        int modeSimilitude = getModeSimilitude(mode);

        init_metadata = computeThreshold(modeSimilitude, edge_factor);
        if (init_metadata == null)
            throw new RuntimeException("initGraphCsv : Une erreur est survenue lors du calcul des seuils.");

        double recommendedThreshold = init_metadata.getEdgeThreshold();
        double recommendedAntiThreshold = init_metadata.getAntiThreshold();

        System.out.println("Recommended threshold: " + recommendedThreshold);
        System.out.println("Recommended anti-threshold: " + recommendedAntiThreshold);

        // Déterminer le mode de détection de communautés à utiliser
        if (community == null)
            throw new RuntimeException("initGraphCsv : Mode de détection de communautés non spécifié.");
        int modeCommunity = getModeCommunity(community);

        metadata = initializeGraph(modeCommunity, recommendedThreshold, recommendedAntiThreshold);

        return data;
    }

	/**
	 * Initialise le graphe avec les données du fichier .dot
	 * @param path Chemin du fichier .dot à charger
	 * @param community Mode de détection de communautés à utiliser
	 * @see GraphData.SimilitudeMode
	 * @see GraphData.NodeCommunity
	 */
	@Override
	public void initGraphDot(String path, GraphData.NodeCommunity community) {
		if (path == null || path.isEmpty())
			throw new RuntimeException("initGraphDot : Chemin du fichier non spécifié.");

		// Déterminer le mode de détection de communautés à utiliser
		if (community == null)
			throw new RuntimeException("initGraphDot : Mode de détection de communautés non spécifié.");
		int modeCommunity = getModeCommunity(community);

		metadata = initializeDot(path, modeCommunity);
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
        //setDimension(WIDTH, HEIGHT); // TODO (la méthode setDimension ne fonctionne pas correctement dans le code C)
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
        
        // Utiliser Platform.runLater pour s'assurer que les modifications
        // des propriétés JavaFX sont faites sur le thread JavaFX
        Platform.runLater(() -> {
            // Définir le nouveau mode
            isRunMode.set(mode == GraphData.GraphMode.RUN);
            isSelectionMode.set(mode == GraphData.GraphMode.SELECTION);
            isMoveMode.set(mode == GraphData.GraphMode.MOVE);
            isDeleteMode.set(mode == GraphData.GraphMode.DELETE);
            
            // Afficher un message selon le mode activé
            if (mode == GraphData.GraphMode.RUN) {
                System.out.println("Switch to RUN");
            } else if (mode == GraphData.GraphMode.SELECTION) {
                System.out.println("Switch to SELECTION - You can select and move vertices");
            } else if (mode == GraphData.GraphMode.MOVE) {
                System.out.println("Switch to MOVE - You can move through the graph");
            } else if (mode == GraphData.GraphMode.DELETE) {
                System.out.println("Switch to DELETE - You can delete vertices");
            }
        });
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
     * @return le seuil actuel pour les arêtes
     */
    @Override
    public double getThreshold() {
        if (init_metadata == null)
            throw new RuntimeException("getThreshold : Métadonnées non initialisées. Veuillez appeler initGraph() avant.");
        return metadata.getEdgeThreshold();
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
     * @return le seuil actuel pour les anti-arêtes
     */
    @Override
    public double getAntiThreshold() {
        if (metadata == null)
            throw new RuntimeException("getAntiThreshold : Métadonnées non initialisées. Veuillez appeler initGraph() avant.");
        return metadata.getAntiThreshold();
    }

	/**
	 * Le seuil de stabilité indique quand le graphe doit s'arrêter (si le mouvement est inférieur au seuil et que suffisamment de temps s'est écoulé, alors le graphe s'arrête de bouger)
	 * @param threshold Nouveau seuil à appliquer
	 */
	@Override
	public void setStabilizedThreshold(double threshold) {
		setThresholdS(threshold);
	}

	/**
	 * Le seuil d'attraction correspond à la distance minimum pour appliquer une force d'attraction entre deux points
	 * @param threshold Seuil d'attraction entre les sommets
	 */
	@Override
	public void setAttractionThreshold(double threshold) {
		setThresholdA(threshold);
	}

	/**
	 * @param freq Fréquence à laquelle les clusters sont mis à jour
	 */
	@Override
	public void setUpdatedFrequence(int freq) {
		setSaut(freq);
	}

	/**
	 * @param friction Friction à appliquer
	 */
	@Override
	public void setNewFriction(double friction) {
		setFriction(friction);
	}

	/**
	 * Choisir le mode de répulsion à utiliser pour mettre à jour les positions
	 * @param mode Mode de répulsion à utiliser
	 * @see GraphData.RepulsionMode
	 */
	@Override
	public void setRepulsionMode(GraphData.RepulsionMode mode) {
		setModeRepulsion(getModeRepulsion(mode));
	}

	/**
	 * @param antiedge_repulsion Force de répulsion des anti-arêtes
	 */
	@Override
	public void setAntiEdgesRepulsion(double antiedge_repulsion) {
		setAntiRepulsion(antiedge_repulsion);
	}

	/**
	 * @param attraction_coeff Force d'attraction entre les sommets
	 */
	@Override
	public void setAttractionCoefficient(double attraction_coeff) {
		setAttractionCoeff(attraction_coeff);
	}

	/**
	 * @param threshold Seuil de répulsion entre les sommets
	 */
	@Override
	public void setRepulsionThreshold(double threshold) {
		setSeuilRep(threshold);
	}

	/**
	 * @param amortissement Amortissement à appliquer (facteur dictant comment la friction évolue après chaque mise à jour du graphe)
	 */
	@Override
	public void setNewAmortissement(double amortissement) {
		setAmortissement(amortissement);
	}

	/**
	 * @param new_number_of_clusters Nombre de clusters à considérer
	 */
	@Override
	public void setNbClusters(int new_number_of_clusters) {
		SetNumberClusters(new_number_of_clusters);
	}

	/**
	 * @param isEnabled <code>true</code> pour utiliser les Kmeans, <code>false</code> pour utiliser le grid clustering
	 */
	@Override
	public void enableKmeans(boolean isEnabled) {
		setKmeansMode(isEnabled);
	}

	/**
	 * @return l'histogramme
	 */
	@Override
	public int[] getHistogramme() {
		return getHistogram();
	}

	/**
	 * @return le degré minimum des sommets à afficher
	 */
	@Override
	public int getMinimumDegree() {
		return minimumDegree.get();
	}

	/**
	 * Affiche les sommets dont le degré est supérieur ou égal à degree
	 * @param degree Degré minimum des sommets à afficher
	 */
	@Override
	public void setMinimumDegree(int degree) {
		if (degree < 0)
			throw new RuntimeException("setMiniumDegree : Degré minimum (" + degree + ") non valide.");
		minimumDegree.set(degree);
		for (Vertex v : vertices) {
			v.setVisibility(v.getDegree() >= degree);
		}
	}

	/**
	 * @param vertex_id Identifiant du sommet à supprimer
	 */
	public void removeVertex(int vertex_id) {
		if (vertex_id < 0 || vertex_id >= vertices.size())
			throw new RuntimeException("removeVertex : Identifiant du sommet (" + vertex_id + ") non valide.");
		selectedVertex.delete();
		System.out.println("Deleted vertex: " + selectedVertex);
		deleteNode(selectedVertex.getId());
		SwingUtilities.invokeLater(() -> glWindow.display());
	}




    // -------------------------------------------------------------------------
    // Export du graphe en PNG
    // -------------------------------------------------------------------------

	/**
	 * Planifie l'exportation de l'image afin qu'elle s'effectue dans le thread OpenGL.
	 * Cela garantit que le contexte GL est valide au moment de l'export.
	 * @param filename Nom du fichier PNG à exporter
	 */
	@Override
	public void exportToPng(final String filename) {
	    // Créer le répertoire d'exportation s'il n'existe pas
	    File captureDir = new File(IMAGE_EXPORT_PATH);
	    if (!captureDir.exists()) {
	        if (!captureDir.mkdir()) {
	            System.err.println("Failed to create '" + IMAGE_EXPORT_PATH + "' directory");
	            return;
	        }
	    }

		// Vérifie si on est sur le thread OpenGL
	    if (animator != null) {
			// Demander une action de rendu unique qui exécutera l'exportation sur le thread OpenGL
			glWindow.invoke(true,drawable -> {
				exportToPngAux(drawable.getGL().getGL4(), IMAGE_EXPORT_PATH + "/" + filename);
				return true;
			});
	    } else {
	        System.err.println("Animation is not active, cannot export");
	    }
	}

	/**
	 * Planifie l'exportation de l'image améliorée afin qu'elle s'effectue dans le thread OpenGL.
	 * Cela garantit que le contexte GL est valide au moment de l'export.
	 * @param filename Nom du fichier PNG à exporter
	 */
	@Override
	public void upgradedExportToPng(final String filename) {
		// Créer le répertoire d'exportation s'il n'existe pas
		File captureDir = new File(IMAGE_EXPORT_PATH);
		if (!captureDir.exists()) {
			if (!captureDir.mkdir()) {
				System.err.println("Failed to create '" + IMAGE_EXPORT_PATH + "' directory");
				return;
			}
		}

		// Vérifie si on est sur le thread OpenGL
		if (animator != null) {
			// Demander une action de rendu unique qui exécutera l'exportation sur le thread OpenGL
			glWindow.invoke(true, drawable -> {
				upgradedExportToPngAux(drawable.getGL().getGL4(), IMAGE_EXPORT_PATH + "/" + filename);
				return true;
			});

		} else {
			System.err.println("Animation is not active, cannot export");
		}
	}

	/**
	 * Exporte le rendu OpenGL actuel vers une image PNG sans perturber l’état d’affichage courant. Crée un framebuffer hors écran pour le rendu.
	 * @param gl Contexte OpenGL
	 * @param path Chemin du fichier PNG à exporter
	 */
	private void exportToPngAux(GL4 gl, String path) {
		System.out.println("Exportation in progress. Please wait...");

	    // Store current viewport dimensions
	    int[] viewport = new int[4];
	    gl.glGetIntegerv(GL4.GL_VIEWPORT, viewport, 0);
	    int viewportWidth = viewport[2];
	    int viewportHeight = viewport[3];

		// Remember the current framebuffer
	    int[] currentFBO = new int[1];
	    gl.glGetIntegerv(GL4.GL_FRAMEBUFFER_BINDING, currentFBO, 0);

	    // Arrays to hold generated OpenGL objects
	    int[] fbo = new int[1];
	    int[] textureId = new int[1];
	    int[] rbo = new int[1];

	    try {
	        // Create a framebuffer object (FBO)
	        gl.glGenFramebuffers(1, fbo, 0);
	        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, fbo[0]);

	        // Create a texture to render to
	        gl.glGenTextures(1, textureId, 0);
	        gl.glBindTexture(GL4.GL_TEXTURE_2D, textureId[0]);

	        // Make sure we use a supported internal format
	        gl.glTexImage2D(GL4.GL_TEXTURE_2D, 0, GL4.GL_RGBA8,
					viewportWidth, viewportHeight,
	                        0, GL4.GL_RGBA, GL4.GL_UNSIGNED_BYTE, null);

	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_MIN_FILTER, GL4.GL_LINEAR);
	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_MAG_FILTER, GL4.GL_LINEAR);
	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_WRAP_S, GL4.GL_CLAMP_TO_EDGE);
	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_WRAP_T, GL4.GL_CLAMP_TO_EDGE);

	        // Attach the texture to the FBO
	        gl.glFramebufferTexture2D(GL4.GL_FRAMEBUFFER, GL4.GL_COLOR_ATTACHMENT0,
	                                  GL4.GL_TEXTURE_2D, textureId[0], 0);

	        // Create a renderbuffer object for depth
	        gl.glGenRenderbuffers(1, rbo, 0);
	        gl.glBindRenderbuffer(GL4.GL_RENDERBUFFER, rbo[0]);
	        gl.glRenderbufferStorage(GL4.GL_RENDERBUFFER, GL4.GL_DEPTH_COMPONENT24,
					viewportWidth, viewportHeight);

	        // Attach the renderbuffer to the FBO
	        gl.glFramebufferRenderbuffer(GL4.GL_FRAMEBUFFER, GL4.GL_DEPTH_ATTACHMENT,
	                                     GL4.GL_RENDERBUFFER, rbo[0]);

	        // Explicitly define which draw buffers to use
	        int[] drawBuffers = {GL4.GL_COLOR_ATTACHMENT0};
	        gl.glDrawBuffers(1, drawBuffers, 0);

	        // Check if framebuffer is complete
	        int status = gl.glCheckFramebufferStatus(GL4.GL_FRAMEBUFFER);
	        if (status != GL4.GL_FRAMEBUFFER_COMPLETE) {
	            System.err.println("Framebuffer is not complete! Status: 0x" + Integer.toHexString(status));

	            // Display more details about the error
	            switch(status) {
	                case GL4.GL_FRAMEBUFFER_UNDEFINED:
	                    System.err.println("GL_FRAMEBUFFER_UNDEFINED");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER");
	                    break;
	                case GL4.GL_FRAMEBUFFER_UNSUPPORTED:
	                    System.err.println("GL_FRAMEBUFFER_UNSUPPORTED");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS");
	                    break;
	                case 0:
	                    System.err.println("Status is 0, which indicates a previous OpenGL error");
	                    break;
	                default:
	                    System.err.println("Unknown framebuffer status error");
	            }
	            return;
	        }

	        // Clear the framebuffer with background
	        gl.glViewport(0, 0, viewportWidth, viewportHeight);
	        gl.glClearColor(bg_color_r, bg_color_g, bg_color_b, 1.0f);
	        gl.glClear(GL4.GL_COLOR_BUFFER_BIT | GL4.GL_DEPTH_BUFFER_BIT);

	        // Update projection matrix
	        updateProjectionMatrix();

	        // Prepare data for rendering
	        prepareVertexRenderData();
	        prepareEdgeRenderData();

	        // Render the graph to the offscreen buffer
	        renderVertices(gl);
	        renderEdges(gl);
	        
	        gl.glFinish();

	        // Read the pixels from the framebuffer
	        ByteBuffer buffer = ByteBuffer.allocateDirect(viewportWidth * viewportHeight * 4).order(ByteOrder.nativeOrder());
	        gl.glReadBuffer(GL4.GL_COLOR_ATTACHMENT0);
	        gl.glReadPixels(0, 0, viewportWidth, viewportHeight, GL4.GL_RGBA, GL4.GL_UNSIGNED_BYTE, buffer);

	        // Flip the image vertically (OpenGL has origin at bottom left, most image formats at top left)
	        BufferedImage image = new BufferedImage(viewportWidth, viewportHeight, BufferedImage.TYPE_INT_ARGB);
	        byte[] row = new byte[viewportWidth * 4];
	        for (int y = 0; y < viewportHeight; y++) {
	            int rowStart = (viewportHeight - 1 - y) * viewportWidth * 4;
	            buffer.position(rowStart);
	            buffer.get(row);
	            for (int x = 0; x < viewportWidth; x++) {
	                int i = x * 4;
	                int r = row[i] & 0xFF;
	                int g = row[i + 1] & 0xFF;
	                int b = row[i + 2] & 0xFF;
	                int a = row[i + 3] & 0xFF;
	                int argb = (a << 24) | (r << 16) | (g << 8) | b;
	                image.setRGB(x, y, argb);
	            }
	        }

	        try {
	            File outputFile = new File(path);
	            ImageIO.write(image, "png", outputFile);
	            System.out.println("Image successfully exported to " + path);
	        } catch (IOException e) {
	            System.err.println("Error writing PNG file: " + e.getMessage());
	            throw new RuntimeException(e);
	        }

	    } catch (Exception e) {
	        System.err.println("Exception in exportToPng: " + e.getMessage());
			throw new RuntimeException(e);
	    } finally {
	        // Clean up resources
	        if (textureId[0] > 0) {
	            gl.glDeleteTextures(1, textureId, 0);
	        }
	        if (rbo[0] > 0) {
	            gl.glDeleteRenderbuffers(1, rbo, 0);
	        }
	        if (fbo[0] > 0) {
	            gl.glDeleteFramebuffers(1, fbo, 0);
	        }

	        // Restore original framebuffer
	        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, currentFBO[0]);

	        // Restore viewport
	        gl.glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	    }
	}

	/**
	 * Exporte le rendu OpenGL actuel vers une image améliorée PNG sans perturber l’état d’affichage courant. Crée un framebuffer hors écran pour le rendu.
	 * @param gl Contexte OpenGL
	 * @param path Chemin du fichier PNG à exporter
	 */
	private void upgradedExportToPngAux(GL4 gl, String path) {
		System.out.println("Upgraded exportation in progress. Please wait...");

	    // Store current viewport dimensions
	    int[] viewport = new int[4];
	    gl.glGetIntegerv(GL4.GL_VIEWPORT, viewport, 0);
	    int viewportWidth = viewport[2];
	    int viewportHeight = viewport[3];

	    // Set the export size for higher resolution
	    int exportWidth = viewportWidth * 15;
	    int exportHeight = viewportHeight * 15;

	    // Remember the current framebuffer
	    int[] currentFBO = new int[1];
	    gl.glGetIntegerv(GL4.GL_FRAMEBUFFER_BINDING, currentFBO, 0);
	    
	    // Store current blend state
	    byte[] blendEnabled = new byte[1];
	    gl.glGetBooleanv(GL4.GL_BLEND, blendEnabled, 0);
	    
	    // Store current blend functions
	    int[] blendSrc = new int[1];
	    int[] blendDst = new int[1];
	    gl.glGetIntegerv(GL4.GL_BLEND_SRC_ALPHA, blendSrc, 0);
	    gl.glGetIntegerv(GL4.GL_BLEND_DST_ALPHA, blendDst, 0);

	    // Arrays to hold generated OpenGL objects
	    int[] fbo = new int[1];
	    int[] textureId = new int[1];
	    int[] rbo = new int[1];

	    try {
	        // Enable alpha blending specifically for the export
	        gl.glEnable(GL4.GL_BLEND);
	        gl.glBlendFunc(GL4.GL_SRC_ALPHA, GL4.GL_ONE_MINUS_SRC_ALPHA);
	        
	        // Create a framebuffer object (FBO)
	        gl.glGenFramebuffers(1, fbo, 0);
	        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, fbo[0]);

	        // Create a texture to render to
	        gl.glGenTextures(1, textureId, 0);
	        gl.glBindTexture(GL4.GL_TEXTURE_2D, textureId[0]);

	        // Make sure we use a supported internal format
	        gl.glTexImage2D(GL4.GL_TEXTURE_2D, 0, GL4.GL_RGBA8,
	                        exportWidth, exportHeight,
	                        0, GL4.GL_RGBA, GL4.GL_UNSIGNED_BYTE, null);

	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_MIN_FILTER, GL4.GL_LINEAR);
	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_MAG_FILTER, GL4.GL_LINEAR);
	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_WRAP_S, GL4.GL_CLAMP_TO_EDGE);
	        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_WRAP_T, GL4.GL_CLAMP_TO_EDGE);

	        // Attach the texture to the FBO
	        gl.glFramebufferTexture2D(GL4.GL_FRAMEBUFFER, GL4.GL_COLOR_ATTACHMENT0,
	                                  GL4.GL_TEXTURE_2D, textureId[0], 0);

	        // Create a renderbuffer object for depth
	        gl.glGenRenderbuffers(1, rbo, 0);
	        gl.glBindRenderbuffer(GL4.GL_RENDERBUFFER, rbo[0]);
	        gl.glRenderbufferStorage(GL4.GL_RENDERBUFFER, GL4.GL_DEPTH_COMPONENT24,
	                                 exportWidth, exportHeight);

	        // Attach the renderbuffer to the FBO
	        gl.glFramebufferRenderbuffer(GL4.GL_FRAMEBUFFER, GL4.GL_DEPTH_ATTACHMENT,
	                                     GL4.GL_RENDERBUFFER, rbo[0]);

	        // Explicitly define which draw buffers to use
	        int[] drawBuffers = {GL4.GL_COLOR_ATTACHMENT0};
	        gl.glDrawBuffers(1, drawBuffers, 0);

	        // Check if framebuffer is complete
	        int status = gl.glCheckFramebufferStatus(GL4.GL_FRAMEBUFFER);
	        if (status != GL4.GL_FRAMEBUFFER_COMPLETE) {
	            System.err.println("Framebuffer is not complete! Status: 0x" + Integer.toHexString(status));

	            // Display more details about the error
	            switch(status) {
	                case GL4.GL_FRAMEBUFFER_UNDEFINED:
	                    System.err.println("GL_FRAMEBUFFER_UNDEFINED");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER");
	                    break;
	                case GL4.GL_FRAMEBUFFER_UNSUPPORTED:
	                    System.err.println("GL_FRAMEBUFFER_UNSUPPORTED");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE");
	                    break;
	                case GL4.GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
	                    System.err.println("GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS");
	                    break;
	                case 0:
	                    System.err.println("Status is 0, which indicates a previous OpenGL error");
	                    break;
	                default:
	                    System.err.println("Unknown framebuffer status error");
	            }
	            return;
	        }

	        // Clear the framebuffer with  background
	        gl.glViewport(0, 0, exportWidth, exportHeight);
	        gl.glClearColor(bg_color_r, bg_color_g, bg_color_b, 1.0f);
	        gl.glClear(GL4.GL_COLOR_BUFFER_BIT | GL4.GL_DEPTH_BUFFER_BIT);

	        // Render the graph to the offscreen buffer
	        // Update projection matrix
	        updateProjectionMatrix();

	        // Prepare data for rendering
	        prepareBezierRenderData();

		renderDoubleCircles(gl);
	        
	        renderBezierCurves(gl);
	        
	        gl.glFinish();

	        // Read the pixels from the framebuffer
	        ByteBuffer buffer = ByteBuffer.allocateDirect(exportWidth * exportHeight * 4).order(ByteOrder.nativeOrder());
	        gl.glReadBuffer(GL4.GL_COLOR_ATTACHMENT0);
	        gl.glReadPixels(0, 0, exportWidth, exportHeight, GL4.GL_RGBA, GL4.GL_UNSIGNED_BYTE, buffer);

	        // Flip the image vertically (OpenGL has origin at bottom left, most image formats at top left)
	        BufferedImage image = new BufferedImage(exportWidth, exportHeight, BufferedImage.TYPE_INT_ARGB);
	        byte[] row = new byte[exportWidth * 4];
	        for (int y = 0; y < exportHeight; y++) {
	            int rowStart = (exportHeight - 1 - y) * exportWidth * 4;
	            buffer.position(rowStart);
	            buffer.get(row);
	            for (int x = 0; x < exportWidth; x++) {
	                int i = x * 4;
	                int r = row[i] & 0xFF;
	                int g = row[i + 1] & 0xFF;
	                int b = row[i + 2] & 0xFF;
	                int a = row[i + 3] & 0xFF;
	                int argb = (a << 24) | (r << 16) | (g << 8) | b;
	                image.setRGB(x, y, argb);
	            }
	        }

	        try {
	            File outputFile = new File(path);
	            ImageIO.write(image, "png", outputFile);
				System.out.println("Upgraded image successfully exported to " + path);
	        } catch (IOException e) {
	            System.err.println("Error writing PNG file: " + e.getMessage());
				throw new RuntimeException(e);
	        }

	    } catch (Exception e) {
	        System.err.println("Exception in exportToPng: " + e.getMessage());
			throw new RuntimeException(e);
	    } finally {
	        // Clean up resources
	        if (textureId[0] > 0) {
	            gl.glDeleteTextures(1, textureId, 0);
	        }
	        if (rbo[0] > 0) {
	            gl.glDeleteRenderbuffers(1, rbo, 0);
	        }
	        if (fbo[0] > 0) {
	            gl.glDeleteFramebuffers(1, fbo, 0);
	        }
	        
	        // Restore original blend state
	        if (blendEnabled[0] != 0) {
	            gl.glEnable(GL4.GL_BLEND);
	        } else {
	            gl.glDisable(GL4.GL_BLEND);
	        }
	        
	        // Restore original blend functions
	        gl.glBlendFunc(blendSrc[0], blendDst[0]);

	        // Restore original framebuffer
	        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, currentFBO[0]);

	        // Restore viewport
	        gl.glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	    }
	}
	
	
	
	
    // -------------------------------------------------------------------------
    // Outils
    // -------------------------------------------------------------------------

    /**
     * @param mode Mode de similitude
     * @return l'identifiant du mode de similitude
	 * @see GraphData.SimilitudeMode
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
	 * @see GraphData.NodeCommunity
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

	/**
	 * @param mode Mode de répulsion
	 * @return l'identifiant du mode de répulsion
	 */
	private static int getModeRepulsion(GraphData.RepulsionMode mode) {
		int modeRepulsion = -1;
		switch (mode) {
			case GraphData.RepulsionMode.REPULSION_BY_DEGREE -> modeRepulsion = 0;
			case GraphData.RepulsionMode.REPULSION_BY_EDGES -> modeRepulsion = 1;
			case GraphData.RepulsionMode.REPULSION_BY_COMMUNITIES -> modeRepulsion = 2;
		}
		if (modeRepulsion == -1)
			throw new RuntimeException("Mode de répulsion non reconnu.");
		return modeRepulsion;
	}

    
    
    
    // -------------------------------------------------------------------------
    // Gestion des buffers
    // -------------------------------------------------------------------------

	/**
	 * Crée les vertex buffers OpenGL nécessaires pour stocker les données des sommets : positions, tailles, couleurs et visibilité.
	 * @param gl Contexte OpenGL utilisé pour allouer les buffers.
	 */
	private void createVertexBuffers(GL4 gl) {
	    // Créer les buffers
	    int[] buffers = new int[4]; // positions, tailles, couleurs, visibilités
	    gl.glGenBuffers(4, buffers, 0);

	    vertexBuffer = buffers[0];
	    vertexSizeBuffer = buffers[1];
	    vertexColorBuffer = buffers[2];
	    vertexVisibilityBuffer = buffers[3];

	    // Buffer pour les positions des sommets
	    FloatBuffer vertexData = FloatBuffer.wrap(vertexPoints);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) vertexData.limit() * Float.BYTES, vertexData, GL4.GL_STATIC_DRAW);

	    // Buffer pour les tailles des sommets
	    FloatBuffer sizeData = FloatBuffer.wrap(vertexSizes);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexSizeBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) sizeData.limit() * Float.BYTES, sizeData, GL4.GL_STATIC_DRAW);

	    // Buffer pour les couleurs des sommets
	    FloatBuffer colorData = FloatBuffer.wrap(vertexColors);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexColorBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) colorData.limit() * Float.BYTES, colorData, GL4.GL_STATIC_DRAW);

	    // Buffer pour la visibilité des sommets
	    FloatBuffer visibilityData = FloatBuffer.wrap(vertexVisibility);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexVisibilityBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) visibilityData.limit() * Float.BYTES, visibilityData, GL4.GL_STATIC_DRAW);
	}

	/**
	 * Crée les buffers OpenGL associés aux arêtes : positions, couleurs, tailles et visibilité. Les buffers sont initialisés sans données (null), car ils seront remplis dynamiquement.
	 * @param gl Contexte OpenGL utilisé pour allouer les buffers.
	 */
	private void createEdgeBuffers(GL4 gl) {
	    int[] buffers = new int[4];
	    gl.glGenBuffers(4, buffers, 0);
	    
	    edgeBuffer = buffers[0];
	    edgeColorBuffer = buffers[1];
	    edgeSizeBuffer = buffers[2];
	    edgeVisibilityBuffer = buffers[3];
	    
	    // La taille des buffers doit correspondre à la quantité de données
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgePoints.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeColorBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgeColors.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeSizeBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgeSizes.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeVisibilityBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgeVisibility.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	}

	/**
	 * Crée les buffers OpenGL utilisés pour représenter les courbes de Bézier associées aux arêtes. Les buffers incluent les points de contrôle, les couleurs, tailles et visibilités, et sont initialisés pour un usage dynamique.
	 * @param gl Contexte OpenGL utilisé pour allouer les buffers.
	 */
	private void createBezierBuffers(GL4 gl) {
	    int[] buffers = new int[4];
	    gl.glGenBuffers(4, buffers, 0);
	    
	    bezierBuffer = buffers[0];
	    bezierColorBuffer = buffers[1];
	    bezierSizeBuffer = buffers[2];
	    bezierVisibilityBuffer = buffers[3];
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) bezierPoints.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierColorBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) bezierColors.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierSizeBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) bezierSizes.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierVisibilityBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) bezierVisibility.length * Float.BYTES, null, GL4.GL_DYNAMIC_DRAW);
	}

	/**
	 * Initialise les tableaux de données nécessaires pour les sommets, les arêtes et les courbes de Bézier. Alloue l’espace mémoire en fonction du nombre de sommets et d’arêtes présents.
	 */
	private void initializeArrays() {
	    int vertexCount = vertices.size();
	    int edgeCount = edges.size();
	    
	    vertexPoints = new float[vertexCount * 2];   // x, y pour chaque sommet
	    vertexSizes = new float[vertexCount];        // taille pour chaque sommet
	    vertexColors = new float[vertexCount * 3];   // RGB pour chaque sommet
	    vertexVisibility = new float[vertexCount];   // visibilité pour chaque sommet

	    // Initialisation par défaut des sommets visibles
	    for (int i = 0; i < vertexCount; i++) {
	        vertexVisibility[i] = 1.0f;
	    }
	    
	    // Pour les arêtes, nous avons 2 points par arête
	    edgePoints = new float[edgeCount * 4];       // x1,y1,x2,y2 pour chaque arête
	    edgeColors = new float[edgeCount * 6];       // 3 composantes (RGB) pour chaque point de l'arête
	    edgeSizes = new float[edgeCount * 2];        // taille pour chaque point de l'arête
	    edgeVisibility = new float[edgeCount * 2];   // visibilité pour chaque point de l'arête
	    
	    // Initialisation des valeurs par défaut
	    for (int i = 0; i < edgeCount; i++) {
	        edgeVisibility[i * 2] = 1.0f;
	        edgeVisibility[i * 2 + 1] = 1.0f;
	    }
	    
	    // Courbe de Bezier : ajout d'un point de contrôle par arête
	    bezierPoints = new float[edgeCount * 6];
	    bezierColors = new float[edgeCount * 3];
	    bezierSizes = new float[edgeCount];
	    bezierVisibility = new float[edgeCount];
	}

	/**
	 * Prépare les données de rendu des sommets en remplissant les tableaux des positions, tailles, couleurs RGB et visibilité à partir des objets {@code Vertex} et de leurs communautés associées.
	 */
	private void prepareVertexRenderData() {
	    Vertex currentVertex;
	    Community currentCommunity;

	    for (int i = 0; i < vertices.size(); i++) {
	        currentVertex = vertices.get(i);
	        currentCommunity = currentVertex.getCommunity();  // léger nettoyage

	        // Mise à jour des buffers
	        vertexPoints[i * 2] = (float) currentVertex.getX();
	        vertexPoints[i * 2 + 1] = (float) currentVertex.getY();
	        vertexSizes[i] = (float) currentVertex.getDiameter();
	        vertexColors[i * 3] = currentCommunity.getR();
	        vertexColors[i * 3 + 1] = currentCommunity.getG();
	        vertexColors[i * 3 + 2] = currentCommunity.getB();

	        // Mise à jour de la visibilité
	        vertexVisibility[i] = (currentVertex.isDeleted() || !currentVertex.isVisible()) ? 0.0f : 1.0f;
	    }
	}

	/**
	 * Prépare les données de rendu des arêtes pour l'affichage. Calcule les positions des extrémités, les couleurs moyennes entre les communautés connectées, les tailles en fonction du poids et définit la visibilité. Seules les arêtes dépassant un seuil de corrélation sont prises en compte.
	 */
	private void prepareEdgeRenderData() {
	    for (int i = 0; i < edges.size(); i++) {
	        Edge currentEdge = edges.get(i);
	        
	    	if (currentEdge.getWeight() > CORRELATION_THRESHOLD) {
		        Vertex startVertex = currentEdge.getStart();
		        Vertex endVertex = currentEdge.getEnd();
		        Community startCommunity = startVertex.getCommunity();
		        Community endCommunity = endVertex.getCommunity();
	
		        // Points de début de l'arête
		        edgePoints[i * 4] = (float) startVertex.getX();
		        edgePoints[i * 4 + 1] = (float) startVertex.getY();
		        
		        // Points de fin de l'arête
		        edgePoints[i * 4 + 2] = (float) endVertex.getX();
		        edgePoints[i * 4 + 3] = (float) endVertex.getY();
		        
		        // Couleur moyenne entre les deux communautés
		        float r = (startCommunity.getR() + endCommunity.getR()) / 2.0f;
		        float g = (startCommunity.getG() + endCommunity.getG()) / 2.0f;
		        float b = (startCommunity.getB() + endCommunity.getB()) / 2.0f;
		        
		        // Couleur pour les deux points de l'arête
		        edgeColors[i * 6] = r;     // Début R
		        edgeColors[i * 6 + 1] = g; // Début G
		        edgeColors[i * 6 + 2] = b; // Début B
		        edgeColors[i * 6 + 3] = r; // Fin R
		        edgeColors[i * 6 + 4] = g; // Fin G
		        edgeColors[i * 6 + 5] = b; // Fin B
		        
		        // Taille pour les deux points
		        float size = (float) currentEdge.getWeight();
		        edgeSizes[i * 2] = size;
		        edgeSizes[i * 2 + 1] = size;
		        
		        // Visibilité - 0.0 si supprimé, 1.0 si visible
		        boolean isHidden = 
		        	    startVertex.isDeleted() || 
		        	    endVertex.isDeleted() || 
		        	    !startVertex.isVisible() || 
		        	    !endVertex.isVisible();
		        float visibility = isHidden ? 0.0f : 1.0f;
		        edgeVisibility[i * 2] = visibility;
		        edgeVisibility[i * 2 + 1] = visibility;
		    }
	    }
	}

	/**
	 * Prépare les données de rendu pour les courbes de Bézier représentant les arêtes. Calcule les points de début, de fin, ainsi qu’un point de contrôle pour la courbure. Assigne des couleurs, des tailles et des niveaux de visibilité. Les courbes sont uniquement créées pour les arêtes significatives.
	 */
	private void prepareBezierRenderData() {
	    for (int i = 0; i < edges.size(); i++) {
	        Edge currentEdge = edges.get(i);
	        
	    	if (currentEdge.getWeight() > CORRELATION_THRESHOLD) {
		        Vertex startVertex = currentEdge.getStart();
		        Vertex endVertex = currentEdge.getEnd();
		        Community startCommunity = startVertex.getCommunity();
		        Community endCommunity = endVertex.getCommunity();
	
		        // Points de début de l'arête
		        bezierPoints[i * 6] = (float) startVertex.getX();
		        bezierPoints[i * 6 + 1] = (float) startVertex.getY();
		        
		        // Calculer le point de contrôle
		        float bezierFactor = 0.2f; // Facteur qui détermine l'amplitude de la courbure
		        float dx = (float) (endVertex.getX() - startVertex.getX());
		        float dy = (float) (endVertex.getY() - startVertex.getY());

		        // Définir un point de contrôle qui dévie légèrement de la ligne
		        float controlX = (float) (startVertex.getX() + (dx / 2) - bezierFactor * dy);
		        float controlY = (float) (startVertex.getY() + (dy / 2) + bezierFactor * dx);

		        // Maintenant, place ce point de contrôle dans ton tableau de points d'arêtes
		        bezierPoints[i * 6 + 2] = controlX; // Calculé plus haut
		        bezierPoints[i * 6 + 3] = controlY;
		        
		        // Points de fin de l'arête
		        bezierPoints[i * 6 + 4] = (float) endVertex.getX();
		        bezierPoints[i * 6 + 5] = (float) endVertex.getY();
		        
		        // Couleur moyenne entre les deux communautés
		        float r = (startCommunity.getR() + endCommunity.getR()) / 2.0f;
		        float g = (startCommunity.getG() + endCommunity.getG()) / 2.0f;
		        float b = (startCommunity.getB() + endCommunity.getB()) / 2.0f;
		        
		        // Couleur pour les trois points de la courbe
	            bezierColors[i * 3] = r;
	            bezierColors[i * 3 + 1] = g;
	            bezierColors[i * 3 + 2] = b;
		        
		        // Taille pour les trois points
		        float size = (float) currentEdge.getWeight();
		        bezierSizes[i] = size;
		        
		        // Visibilité - 0.0 si supprimé, 1.0 si visible
		        boolean isHidden = 
		        	    startVertex.isDeleted() || 
		        	    endVertex.isDeleted() || 
		        	    !startVertex.isVisible() || 
		        	    !endVertex.isVisible();
		        float visibility = isHidden ? 0.0f : 1.0f;
		        bezierVisibility[i] = visibility;
		    }
	    }
	}
	
	
	
	
    // -------------------------------------------------------------------------
    // Méthodes de rendu
    // -------------------------------------------------------------------------

	/**
	 * Rendu des sommets (points) sur la scène OpenGL.
	 * Cette méthode met à jour les buffers GPU avec les données des sommets, notamment
	 * leur position, taille, couleur et visibilité, puis configure les attributs de
	 * vertex nécessaires pour le shader des points. Enfin, elle effectue le rendu
	 * sous forme de points via `glDrawArrays`.
	 * @param gl Contexte OpenGL utilisé pour effectuer le rendu.
	 */
	private void renderVertices(GL4 gl) {
	    // === Envoi des données GPU ===
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexPoints.length * Float.BYTES, FloatBuffer.wrap(vertexPoints));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexSizeBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexSizes.length * Float.BYTES, FloatBuffer.wrap(vertexSizes));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexColorBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexColors.length * Float.BYTES, FloatBuffer.wrap(vertexColors));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexVisibilityBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexVisibility.length * Float.BYTES, FloatBuffer.wrap(vertexVisibility));


	    // === Affichage des sommets ===
	    gl.glUseProgram(verticesShaderProgram);

	    gl.glEnableVertexAttribArray(0); // position
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexBuffer);
	    gl.glVertexAttribPointer(0, 2, GL4.GL_FLOAT, false, 0, 0);

	    gl.glEnableVertexAttribArray(1); // size
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexSizeBuffer);
	    gl.glVertexAttribPointer(1, 1, GL4.GL_FLOAT, false, 0, 0);

	    gl.glEnableVertexAttribArray(2); // color
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexColorBuffer);
	    gl.glVertexAttribPointer(2, 3, GL4.GL_FLOAT, false, 0, 0);
	    
	    gl.glEnableVertexAttribArray(3); // visibility
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexVisibilityBuffer);
	    gl.glVertexAttribPointer(3, 1, GL4.GL_FLOAT, false, 0, 0);

	    // Matrice de projection (identité ou zoom plus tard)
	    int transformLoc = gl.glGetUniformLocation(verticesShaderProgram, "u_transform");
	    gl.glUniformMatrix4fv(transformLoc, 1, false, projectionMatrix);

	    // Dessin des points
	    gl.glDrawArrays(GL4.GL_POINTS, 0, vertices.size());
	}

	/**
	 * Rendu des arêtes (lignes) entre les sommets dans la scène OpenGL.
	 * Cette méthode transmet les informations des arêtes au GPU : coordonnées des extrémités,
	 * couleurs, tailles et visibilité. Elle configure ensuite les attributs nécessaires
	 * pour le shader dédié aux arêtes, puis dessine les lignes à l’aide de `glDrawArrays`
	 * avec le mode `GL_LINES`.
	 * @param gl Contexte OpenGL utilisé pour effectuer le rendu.
	 */
	private void renderEdges(GL4 gl) {
		// === Envoi des données GPU ===
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgePoints.length * Float.BYTES, FloatBuffer.wrap(edgePoints));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeColorBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeColors.length * Float.BYTES, FloatBuffer.wrap(edgeColors));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeSizeBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeSizes.length * Float.BYTES, FloatBuffer.wrap(edgeSizes));
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeVisibilityBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeVisibility.length * Float.BYTES, FloatBuffer.wrap(edgeVisibility));

	    // === Affichage des arêtes ===
	    gl.glUseProgram(edgesShaderProgram);

	    gl.glEnableVertexAttribArray(0); // position
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeBuffer);
	    gl.glVertexAttribPointer(0, 2, GL4.GL_FLOAT, false, 0, 0);

	    gl.glEnableVertexAttribArray(1); // color
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeColorBuffer);
	    gl.glVertexAttribPointer(1, 3, GL4.GL_FLOAT, false, 0, 0);

	    gl.glEnableVertexAttribArray(2); // size
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeSizeBuffer);
	    gl.glVertexAttribPointer(2, 1, GL4.GL_FLOAT, false, 0, 0);
	    
	    gl.glEnableVertexAttribArray(3); // visibility
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeVisibilityBuffer);
	    gl.glVertexAttribPointer(3, 1, GL4.GL_FLOAT, false, 0, 0);

	    int transformLocEdges = gl.glGetUniformLocation(edgesShaderProgram, "u_transform");
	    gl.glUniformMatrix4fv(transformLocEdges, 1, false, projectionMatrix);

	    // Dessin des lignes (2 points par arête)
	    gl.glDrawArrays(GL4.GL_LINES, 0, edges.size() * 2);
	}

	/**
	 * Rendu d’un motif de cercles concentriques autour des sommets.
	 * Utilise un shader spécifique pour afficher des doubles cercles sur chaque sommet.
	 * Les buffers pour position, taille, couleur et visibilité sont mis à jour puis
	 * associés aux attributs du shader. Le rendu est ensuite effectué via `glDrawArrays`
	 * en mode `GL_POINTS`.
	 * @param gl Contexte OpenGL utilisé pour effectuer le rendu.
	 */
	private void renderDoubleCircles(GL4 gl) {
		gl.glUseProgram(doubleCircleShaderProgram);
        
        gl.glEnableVertexAttribArray(0);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexPoints.length * Float.BYTES, FloatBuffer.wrap(vertexPoints));
        gl.glVertexAttribPointer(0, 2, GL4.GL_FLOAT, false, 0, 0);

        gl.glEnableVertexAttribArray(1);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexSizeBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexSizes.length * Float.BYTES, FloatBuffer.wrap(vertexSizes));
        gl.glVertexAttribPointer(1, 1, GL4.GL_FLOAT, false, 0, 0);

        gl.glEnableVertexAttribArray(2);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexColorBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexColors.length * Float.BYTES, FloatBuffer.wrap(vertexColors));
        gl.glVertexAttribPointer(2, 3, GL4.GL_FLOAT, false, 0, 0);

        gl.glEnableVertexAttribArray(3);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexVisibilityBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexVisibility.length * Float.BYTES, FloatBuffer.wrap(vertexVisibility));
        gl.glVertexAttribPointer(3, 1, GL4.GL_FLOAT, false, 0, 0);

        int transformLoc = gl.glGetUniformLocation(verticesShaderProgram, "u_transform");
        gl.glUniformMatrix4fv(transformLoc, 1, false, projectionMatrix);

        gl.glDrawArrays(GL4.GL_POINTS, 0, vertices.size());
	}

	/**
	 * Rendu de courbes de Bézier quadratiques instanciées.
	 * Cette méthode configure les buffers et les attributs de vertex pour dessiner
	 * des courbes de Bézier définies par trois points de contrôle (p0, p1, p2).
	 * Chaque courbe a une couleur, une taille de trait, et une visibilité propres.
	 * Le rendu se fait par instanciation en utilisant `glDrawArraysInstanced` avec
	 * une interpolation segmentée.
	 * @param gl Contexte OpenGL utilisé pour effectuer le rendu.
	 */
	private void renderBezierCurves(GL4 gl) {
	    gl.glUseProgram(bezierShaderProgram);
	    
	    // Set up the buffer data for the shader
	    // Upload control points to GPU
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) bezierPoints.length * Float.BYTES, FloatBuffer.wrap(bezierPoints));
	    
	    // Configure vertex attributes for the control points
	    // Each curve has 3 control points (p0, p1, p2), each with 2 components (x, y)
	    final int STRIDE = 6 * Float.BYTES; // 6 floats per curve (p0x, p0y, p1x, p1y, p2x, p2y)
	    
	    // Start point (p0)
	    gl.glEnableVertexAttribArray(0);
	    gl.glVertexAttribPointer(0, 2, GL4.GL_FLOAT, false, STRIDE, 0);
	    gl.glVertexAttribDivisor(0, 1); // One value per instance
	    
	    // Control point (p1)
	    gl.glEnableVertexAttribArray(1);
	    gl.glVertexAttribPointer(1, 2, GL4.GL_FLOAT, false, STRIDE, 2 * Float.BYTES);
	    gl.glVertexAttribDivisor(1, 1); // One value per instance
	    
	    // End point (p2)
	    gl.glEnableVertexAttribArray(2);
	    gl.glVertexAttribPointer(2, 2, GL4.GL_FLOAT, false, STRIDE, 4 * Float.BYTES);
	    gl.glVertexAttribDivisor(2, 1); // One value per instance
	    
	    // Color attribute
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierColorBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) bezierColors.length * Float.BYTES, FloatBuffer.wrap(bezierColors));
	    gl.glEnableVertexAttribArray(3);
	    gl.glVertexAttribPointer(3, 3, GL4.GL_FLOAT, false, 0, 0);
	    gl.glVertexAttribDivisor(3, 1); // One color per curve
	    
	    // Size attribute
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierSizeBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) bezierSizes.length * Float.BYTES, FloatBuffer.wrap(bezierSizes));
	    gl.glEnableVertexAttribArray(4);
	    gl.glVertexAttribPointer(4, 1, GL4.GL_FLOAT, false, 0, 0);
	    gl.glVertexAttribDivisor(4, 1); // One size per curve
	    
	    // Visibility attribute
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, bezierVisibilityBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) bezierVisibility.length * Float.BYTES, FloatBuffer.wrap(bezierVisibility));
	    gl.glEnableVertexAttribArray(5);
	    gl.glVertexAttribPointer(5, 1, GL4.GL_FLOAT, false, 0, 0);
	    gl.glVertexAttribDivisor(5, 1); // One visibility value per curve
	    
	    // Set transformation matrix uniform
	    int transformLoc = gl.glGetUniformLocation(bezierShaderProgram, "u_transform");
	    gl.glUniformMatrix4fv(transformLoc, 1, false, projectionMatrix);
	    
	    // Draw multiple instances of the Bézier curve with different parameters
	    final int SEGMENTS = 100; // Number of segments to render each curve with
	    gl.glLineWidth(2f);
	    gl.glDrawArraysInstanced(GL4.GL_LINE_STRIP, 0, SEGMENTS + 1, edges.size());
	    
	    // Disable vertex attributes
	    gl.glDisableVertexAttribArray(0);
	    gl.glDisableVertexAttribArray(1);
	    gl.glDisableVertexAttribArray(2);
	    gl.glDisableVertexAttribArray(3);
	    gl.glDisableVertexAttribArray(4);
	    gl.glDisableVertexAttribArray(5);
	    
	    gl.glVertexAttribDivisor(0, 0);
	    gl.glVertexAttribDivisor(1, 0);
	    gl.glVertexAttribDivisor(2, 0);
	    gl.glVertexAttribDivisor(3, 0);
	    gl.glVertexAttribDivisor(4, 0);
	    gl.glVertexAttribDivisor(5, 0);
	}
	
	
	
	
    // -------------------------------------------------------------------------
    // Shaders
    // -------------------------------------------------------------------------
	
	private static final String POINT_VERTEX_SHADER =
	        """
									#version 400 core
									layout(location = 0) in vec2 position;
									layout(location = 1) in float size;
									layout(location = 2) in vec3 color;
									layout(location = 3) in float visibility;
									uniform mat4 u_transform;
									out vec3 fragColor;
									out float fragVisibility;
									void main() {
												vec4 pos = vec4(position, 0.0, 1.0);
												gl_Position = u_transform * pos;
												gl_PointSize = size;
												fragColor = color;
												fragVisibility = visibility;
									}
									""";

	private static final String POINT_FRAGMENT_SHADER =
	        """
									#version 400 core
									in vec3 fragColor;
									in float fragVisibility;
									out vec4 color;
									void main() {
												if (fragVisibility == 0.0) {
																discard;
												}
												float dist = length(gl_PointCoord - vec2(0.5, 0.5));
												if (dist < 0.5) {
																color = vec4(fragColor, 1.0);
												} else {
																discard;
												}
									}
									""";
	
	private static final String EDGE_VERTEX_SHADER =
			"""
			#version 400 core
			layout(location = 0) in vec2 position;
			layout(location = 1) in vec3 color;
			layout(location = 2) in float size;
			layout(location = 3) in float visibility;
			uniform mat4 u_transform;
			out vec3 fragColor;
			out float fragVisibility;
			void main() {
			vec4 pos = vec4(position, 0.0, 1.0);
			gl_Position = u_transform * pos;
			fragColor = color;
			fragVisibility = visibility;
			}
			""";

	private static final String EDGE_FRAGMENT_SHADER =
			"""
			#version 400 core
			in vec3 fragColor;
			in float fragVisibility;
			out vec4 color;
			void main() {
			if (fragVisibility == 0.0) {
			discard;
			}
			color = vec4(fragColor, 1.0);
			}
			""";

	private static final String DOUBLE_CIRCLE_VERTEX_SHADER =
	        """
			#version 400 core
			layout(location = 0) in vec2 position;
			layout(location = 1) in float size;
			layout(location = 2) in vec3 color;
			layout(location = 3) in float visibility;
			uniform mat4 u_transform;
			out vec3 fragColor;
			out float fragVisibility;
			out float fragPointSize;
			void main() {
							vec4 pos = vec4(position, 0.0, 1.0);
							gl_Position = u_transform * pos;
							gl_PointSize = size;
							fragColor = color;
							fragVisibility = visibility;
							fragPointSize = size;
			}
			""";

	private static final String DOUBLE_CIRCLE_FRAGMENT_SHADER =
	        """
			#version 400 core
			in vec3 fragColor;
			in float fragVisibility;
			in float fragPointSize;
			out vec4 color;
			
			void main() {
			    if (fragVisibility == 0.0) {
			        discard;
			    }
			
			    // Convertir les coordonnées pour qu'elles soient entre -0.5 et 0.5
			    vec2 centeredCoord = gl_PointCoord - vec2(0.5, 0.5);

			    // Distance stricte depuis le centre (test circulaire)
			    float dist = length(centeredCoord);

			    // Rejet strict de tout ce qui est en dehors du cercle
			    if (dist > 0.5) {
			        discard;
			    }

			    // Rayons normalisés (pas de conversion en pixels)
			    float innerRadius = 0.35;
			    float borderWidth = 5.0 / fragPointSize; // 5 pixels convertis en coordonnées normalisées
			    float outerRadius = innerRadius + borderWidth;

			    // Tests de distance simplifiés
			    if (dist < innerRadius) {
			        color = vec4(fragColor, 1.0);
			    }
			    else if (dist < outerRadius) {
			        color = vec4(1.0, 1.0, 1.0, 1.0);
			    }
			    else {
			        discard; // Tout ce qui est au-delà du cercle extérieur est rejeté
			    }
			}
			""";


	private static final String BEZIER_VERTEX_SHADER =
			"""
			#version 400 core
			layout(location = 0) in vec2 p0; // start point
			layout(location = 1) in vec2 p1; // control point
			layout(location = 2) in vec2 p2; // end point
			layout(location = 3) in vec3 color;
			layout(location = 4) in float size;
			layout(location = 5) in float visibility;

			uniform mat4 u_transform;

			out vec3 fragColor;
			out float fragSize;
			out float fragVisibility;

			void main() {
			    // Calculate parameter t based on vertex ID (0 to SEGMENTS)
			    float t = float(gl_VertexID) / 100.0; // For 21 points (0 to 20)

			    // Quadratic Bézier formula: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
			    float u = 1.0 - t;
			    vec2 position = u * u * p0 + 2.0 * u * t * p1 + t * t * p2;

			    // Transform to clip space
			    gl_Position = u_transform * vec4(position, 0.0, 1.0);

			    // Pass attributes to fragment shader
			    fragColor = color;
			    fragSize = size;
			    fragVisibility = visibility;

			    // Set point size if rendering as points
			    gl_PointSize = size;
			}
			""";

	private static final String BEZIER_FRAGMENT_SHADER =
			"""
			#version 400 core
			in vec3 fragColor;
			in float fragSize;
			in float fragVisibility;

			out vec4 outColor;

			void main() {
			    // Discard fragment if visibility is 0
			    if (fragVisibility == 0.0) {
			        discard;
			    }
			
			    // Output color with visibility as alpha
			    outColor = vec4(fragColor, fragVisibility);
			}
			""";



    // -------------------------------------------------------------------------
    // Shader utilities
    // -------------------------------------------------------------------------

	/**
	 * Crée un programme shader OpenGL à partir des sources vertex et fragment fournies.
	 * @param gl Contexte OpenGL utilisé pour créer et gérer les shaders.
	 * @param vertexSource Code source du shader de vertex.
	 * @param fragmentSource Code source du shader de fragment.
	 * @return L'identifiant du programme shader compilé et lié, ou 0 en cas d'échec.
	 */
	private int createShaderProgram(GL4 gl, String vertexSource, String fragmentSource) {
	    // Compiler les shaders
	    int vertexShader = compileShader(gl, GL4.GL_VERTEX_SHADER, vertexSource);
	    int fragmentShader = compileShader(gl, GL4.GL_FRAGMENT_SHADER, fragmentSource);

	    // Créer un programme shader
	    int program = gl.glCreateProgram();
	    gl.glAttachShader(program, vertexShader);
	    gl.glAttachShader(program, fragmentShader);
	    gl.glLinkProgram(program);

	    // Vérifier si le programme a bien été lié
	    IntBuffer linkStatus = IntBuffer.allocate(1);
	    gl.glGetProgramiv(program, GL4.GL_LINK_STATUS, linkStatus);
	    if (linkStatus.get(0) != GL4.GL_TRUE) {
	        System.err.println("Erreur de liaison du programme de shaders.");
	    }

	    return program;
	}

	/**
	 * Compile un shader OpenGL du type spécifié à partir du code source fourni.
	 * @param gl Contexte OpenGL utilisé pour la compilation.
	 * @param type Type de shader à compiler (par exemple GL_VERTEX_SHADER ou GL_FRAGMENT_SHADER).
	 * @param source Code source GLSL du shader.
	 * @return L'identifiant du shader compilé, ou 0 si la compilation échoue.
	 */
	private int compileShader(GL4 gl, int type, String source) {
	    int shader = gl.glCreateShader(type);
	    gl.glShaderSource(shader, 1, new String[]{source}, null);
	    gl.glCompileShader(shader);

	    // Vérification de la compilation
	    IntBuffer compiled = IntBuffer.allocate(1);
	    gl.glGetShaderiv(shader, GL4.GL_COMPILE_STATUS, compiled);
	    if (compiled.get(0) != GL4.GL_TRUE) {
	        System.err.println("Erreur de compilation du shader");
	        System.err.println(getShaderInfoLog(gl, shader));
	    }

	    return shader;
	}

	/**
	 * Récupère le journal d'information (log) d'un shader, typiquement utilisé en cas d'erreur de compilation pour connaître les détails.
	 * @param gl Contexte OpenGL utilisé pour interroger le shader.
	 * @param shader Identifiant du shader concerné.
	 * @return Le contenu du journal d'information du shader sous forme de chaîne de caractères.
	 */
	private String getShaderInfoLog(GL4 gl, int shader) {
	    IntBuffer logLength = IntBuffer.allocate(1);
	    gl.glGetShaderiv(shader, GL4.GL_INFO_LOG_LENGTH, logLength);
	    byte[] log = new byte[logLength.get(0)];
	    gl.glGetShaderInfoLog(shader, logLength.get(0), null, 0, log, 0);
	    return new String(log);
	}

	/**
	 * Met à jour la matrice de projection orthographique en fonction des dimensions de la fenêtre, du niveau de zoom et des décalages de vue.
	 * Cette méthode calcule une matrice de projection adaptée à un rendu 2D ou orthographique, et met à jour le buffer de matrice de projection utilisé dans les shaders.
	 */
	private void updateProjectionMatrix() {
	    float left   = (float) (-WIDTH / 2.0 / zoomFactor + viewOffsetX);
	    float right  = (float) (WIDTH / 2.0 / zoomFactor + viewOffsetX);
	    float bottom = (float) (-HEIGHT / 2.0 / zoomFactor + viewOffsetY);
	    float top    = (float) (HEIGHT / 2.0 / zoomFactor + viewOffsetY);

	    float[] mat = new float[]{
	        2f / (right - left), 0, 0, 0,
	        0, 2f / (top - bottom), 0, 0,
	        0, 0, -1, 0,
	        -(right + left) / (right - left), -(top + bottom) / (top - bottom), 0, 1
	    };

	    projectionMatrix.clear();
	    projectionMatrix.put(mat);
	    projectionMatrix.flip();
	}

}
