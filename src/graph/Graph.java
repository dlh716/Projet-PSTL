package graph;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
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
import com.jogamp.opengl.awt.GLJPanel;
import com.jogamp.opengl.util.FPSAnimator;

import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.property.*;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Region;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
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
    private native double[][] startsProgram(String filename);
	private native Metadata computeThreshold(int modeSimilitude, int edge_factor);
	private native Metadata initiliazeGraph(int modeCommunity, double threshold, double anti_threshold);
	private native Metadata initializeDot(String filepath, int modeCommunity);

	/** the calculation depends on how big the window is
	 * @param width positive real number
	 * @param height positive real number
	 */
	private native void setDimension(double width, double height); // TODO

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
    /**
     * ignores node for the algorithm
     * @param index index of node to delete
     */
	private native void deleteNode(int index);
    /**
     * restores deleted node for the algorithm
     * @param index index of node to restore
     */
	private native void restoreNode(int index);
	private native void setKmeansMode(boolean md);
	private native int[] getHistogram();
	private native void freeAllocatedMemory();
    // freeAllocatedMemory but also frees the data
    private native void freeProgramMemory();




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
    public static int GRAPH_UPSCALE = 5; // Facteur d'agrandissment du graphe

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
    private Vertex selectedVertex = null;
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

    // Variables pour les buffers et shaders
	private int pointsShaderProgram;
	private int edgesShaderProgram;
	private FloatBuffer projectionMatrix;
	private int vertexBuffer;
	private int vertexColorBuffer;
	private int vertexSizeBuffer;
	private int edgeBuffer;
	private int edgeColorBuffer;
	private int edgeSizeBuffer;
	private float[] edgePoints;
	private float[] edgeSizes;
	private float[] edgeColors;
	private float[] vertexPoints;
	private float[] vertexSizes;
	private float[] vertexColors;
	private float[] edgeVisibility;
	private int edgeVisibilityBuffer;

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
     * Méthode principale de l'application
     */
    @Override
    public void start(Stage primaryStage) {

        // Ajouter les listeners pour les différents modes du graphe
    	isRunMode.addListener((obs, oldValue, newValue) -> {
    	    Platform.runLater(() -> {
    	        if (newValue) {
    	            System.out.println("Tentative de reprise de l'animation...");
    	            // Forcer l'état de l'animateur si nécessaire
    	            if (!animator.isAnimating()) {
    	                animator.resume();
    	                System.out.println("Reprise de l'animation réussie");
    	            }
    	        } else {
    	            System.out.println("Tentative de pause de l'animation...");
    	            if (animator.isAnimating()) {
    	                animator.pause();
    	                System.out.println("Pause de l'animation réussie");
    	            }
    	        }
    	    });
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
        
        // Initialisation des buffers pour les sommets et arêtes
        initializeArrays();
        
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
        
        // Tests d'actions sur le graphe (en attendant l'interface graphique)
//        testActions();
        
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
        // Ajuster les coordonnées en fonction du zoom
        double adjustedX = (x - viewOffsetX) / zoomFactor;
        double adjustedY = (y - viewOffsetY) / zoomFactor;

        for (Vertex v : vertices) {
            double dx = adjustedX - v.getX();
            double dy = adjustedY - v.getY();
            double distance = Math.sqrt(dx * dx + dy * dy);
            if (distance <= (v.getDiameter() / 2) + 5) // Ajouter une marge pour faciliter la sélection
                return v;
        }
        return null;
    }
    
    /**
     * Ajoute les listeners pour la souris
     */
    private void addMouseListeners() {

        glWindow.addMouseListener(new MouseListener() {

            // Gère le maintien du clic de la souris
            @Override
            public void mousePressed(MouseEvent e) {
                double x = e.getX() - WIDTH / 2.0 + viewOffsetX;
                double y = HEIGHT / 2.0 - e.getY() + viewOffsetY;

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
                
				// Supprimer un sommet
				else if (isDeleteMode.get() && selectedVertex != null) {
					selectedVertex.delete();
					System.out.println("Sommet supprimé : " + selectedVertex);
					deleteNode(selectedVertex.getId());
					SwingUtilities.invokeLater(() -> glWindow.display());
				}
            }

            // Gère le relâchement du clic de la souris
            @Override
            public void mouseReleased(MouseEvent e) {

				// Déplacer un sommet
                if (isSelectionMode.get() && isDraggingVertex && selectedVertex != null) {
                    isDraggingVertex = false;
					System.out.println("Déplacement du sommet vers (" + selectedVertex.getX() + ", " + selectedVertex.getY() + ")");
                    setNodePosition(selectedVertex.getId(), selectedVertex.getX() / GRAPH_UPSCALE, selectedVertex.getY() / GRAPH_UPSCALE);
                    vertexPoints[selectedVertex.getId() * 2] = (float) selectedVertex.getX();
                    vertexPoints[selectedVertex.getId() * 2 + 1] = (float) selectedVertex.getY();
                    selectedVertex = null;
                }
                isDraggingGraph = false;

            }

            @Override
            public void mouseClicked(MouseEvent e) {
//				double x = e.getX() - WIDTH / 2.0 + viewOffsetX;
//				double y = HEIGHT / 2.0 - e.getY() + viewOffsetY;
//
//				// Vérifier si un sommet est cliqué
//				selectedVertex = findVertexAt(x, y);
//
//				// Obtenir les informations sur un sommet
//				if (isSelectionMode.get() && selectedVertex != null) {
//					System.out.println("Sommet sélectionné : " + selectedVertex);
//				}
//
//				// Supprimer un sommet
//				else if (isDeleteMode.get() && selectedVertex != null) {
//					selectedVertex.delete();
//					System.out.println("Sommet supprimé : " + selectedVertex);
////					for (Edge edge : selectedVertex.getEdges())
////						edge.delete();
//					//setdeleteNode(selectedVertex.getId()); // TODO retourne une erreur
//					SwingUtilities.invokeLater(() -> glWindow.display());
//				}
			}

            @Override
            public void mouseEntered(MouseEvent e) {}

            @Override
            public void mouseExited(MouseEvent e) {}
            

            // Gère le déplacement de la souris
            @Override
            public void mouseDragged(MouseEvent e) {
            	boolean updated = false;
                // Déplacer un sommet
                if (isSelectionMode.get() && isDraggingVertex && selectedVertex != null) {
                    double newX = e.getX() - WIDTH / 2.0 + viewOffsetX + dragOffsetX;
                    double newY = HEIGHT / 2.0 - e.getY() + viewOffsetY + dragOffsetY;
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

            @Override
            public void mouseWheelMoved(MouseEvent e) {
				if (isMoveMode.get()) {
					float[] rotation = e.getRotation(); // [x, y]
					float scrollY = rotation[1];
					double zoomAmount = 1.1;

					if (scrollY == 0) return;

					double mouseXBefore = (e.getX() - WIDTH / 2.0) + viewOffsetX;
					double mouseYBefore = (HEIGHT / 2.0 - e.getY()) + viewOffsetY;

					if (scrollY > 0) {
						zoomFactor *= zoomAmount;
					} else {
						zoomFactor /= zoomAmount;
					}

					// Limite du facteur de zoom pour éviter les zooms extrêmes
					zoomFactor = Math.max(0.1, Math.min(zoomFactor, 10.0));

					double mouseXAfter = (e.getX() - WIDTH / 2.0) + viewOffsetX;
					double mouseYAfter = (HEIGHT / 2.0 - e.getY()) + viewOffsetY;

					viewOffsetX += (mouseXBefore - mouseXAfter);
					viewOffsetY += (mouseYBefore - mouseYAfter);

					updateProjectionMatrix();
					SwingUtilities.invokeLater(() -> glWindow.display());
				}
            }

        });
    }
    
    private void addKeyListeners() {
        glWindow.addKeyListener(new KeyListener() {

        	@Override
        	public void keyPressed(KeyEvent e) {
        	    char keyChar = e.getKeyChar(); // Récupère le caractère associé à la touche pressée

        	    // Afficher le caractère pour le débogage
        	    System.out.println("Touche pressée : " + keyChar);
        	    
        	    // Vérifier si la touche pressée est un chiffre entre 1 et 9
        	    if (keyChar >= '1' && keyChar <= '9') {
        	        int keyNumber = keyChar - '0';  // Convertit le caractère en un nombre entier
        	        System.out.println("Touche " + keyNumber + " pressée");

        	        // Switch en fonction de la touche pressée
        	        switch (keyNumber) {
        	            case 1:
        	                setMode(GraphData.GraphMode.SELECTION);
        	                System.out.println("Switch to " + getMode() + " - Vous pouvez sélectionner et déplacer des sommets");
        	                break;
        	            case 2:
        	                setMode(GraphData.GraphMode.DELETE);
        	                System.out.println("Switch to " + getMode() + " - Vous pouvez supprimer des sommets");
        	                break;
        	            case 3:
        	                setMode(GraphData.GraphMode.RUN);
        	                System.out.println("Back to " + getMode() + " - Exécution du graphe (en mouvement)");
        	                break;
        	            case 4:
        	                setMode(GraphData.GraphMode.MOVE);
        	                System.out.println("Back to " + getMode() + " - Vous pouvez vous déplacer dans le graphe");
        	                break;
        	        }
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
        GL4 gl = drawable.getGL().getGL4(); // Utiliser GL4 au lieu de GL
        gl.glClearColor(bg_color_r, bg_color_g, bg_color_b, 1.0f); // Couleur de fond de l'écran
        gl.glEnable(GL4.GL_DEPTH_TEST); // Activer le test de profondeur pour les objets 3D
	    gl.glEnable(GL4.GL_PROGRAM_POINT_SIZE);
	    gl.glEnable(GL4.GL_BLEND);
	    gl.glBlendFunc(GL4.GL_SRC_ALPHA, GL4.GL_ONE_MINUS_SRC_ALPHA);

	    // Créer des buffers pour les positions, tailles et couleurs
	    createBuffers(gl);

	    String vertexShaderSourcePoints =
				"""
				#version 400 core
				layout(location = 0) in vec2 position;
				layout(location = 1) in float size;
				layout(location = 2) in vec3 color;
				uniform mat4 u_transform;
				out vec3 fragColor;
				void main() {
				   vec4 pos = vec4(position, 0.0, 1.0);
				   gl_Position = u_transform * pos;
				   gl_PointSize = size;
				   fragColor = color;
				}
						""";

	    String fragmentShaderSourcePoints =
				"""
				#version 400 core
				in vec3 fragColor;
				out vec4 color;
				void main() {
				   float dist = length(gl_PointCoord - vec2(0.5, 0.5));
				   if (dist < 0.5) {
				       color = vec4(fragColor, 1.0);
				   } else {
				       discard;
				   }
				}
				""";

	    createEdgeBuffers(gl);

	    String vertexShaderSourceEdges =
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

	    String fragmentShaderSourceEdges =
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

	    pointsShaderProgram = createShaderProgram(gl, vertexShaderSourcePoints, fragmentShaderSourcePoints);
	    edgesShaderProgram = createShaderProgram(gl, vertexShaderSourceEdges, fragmentShaderSourceEdges);
        // gl.glHint(GL4.GL_POINT_SMOOTH_HINT, GL4.GL_NICEST);
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
	    System.out.println("display");

	    // Met à jour la matrice de transformation avec les offsets actuels
	    updateProjectionMatrix();

	    gl.glClear(GL4.GL_COLOR_BUFFER_BIT | GL4.GL_DEPTH_BUFFER_BIT);

        if (isRunMode.get()) {
            boolean is_running = updatePositions();
            List<Vertex> updatedVertices = List.of(getPositions());
            for (int i = 0; i < updatedVertices.size(); i++) {
                Vertex v = vertices.get(i);
                // Mise à jour des coordonnées des sommets
                v.updatePosition(updatedVertices.get(i).getX(), updatedVertices.get(i).getY());
            }
        }

        prepareVertexRenderData();
	    prepareEdgeRenderData();

	    // === Envoi des données GPU ===
	    // Sommets
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexPoints.length * Float.BYTES, FloatBuffer.wrap(vertexPoints));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexSizeBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexSizes.length * Float.BYTES, FloatBuffer.wrap(vertexSizes));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexColorBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexColors.length * Float.BYTES, FloatBuffer.wrap(vertexColors));

	    // Arêtes
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgePoints.length * Float.BYTES, FloatBuffer.wrap(edgePoints));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeColorBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeColors.length * Float.BYTES, FloatBuffer.wrap(edgeColors));

	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeSizeBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeSizes.length * Float.BYTES, FloatBuffer.wrap(edgeSizes));
	    
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeVisibilityBuffer);
	    gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeVisibility.length * Float.BYTES, FloatBuffer.wrap(edgeVisibility));

	    // === Affichage des sommets ===
	    gl.glUseProgram(pointsShaderProgram);

	    gl.glEnableVertexAttribArray(0); // position
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexBuffer);
	    gl.glVertexAttribPointer(0, 2, GL4.GL_FLOAT, false, 0, 0);

	    gl.glEnableVertexAttribArray(1); // size
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexSizeBuffer);
	    gl.glVertexAttribPointer(1, 1, GL4.GL_FLOAT, false, 0, 0);

	    gl.glEnableVertexAttribArray(2); // color
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexColorBuffer);
	    gl.glVertexAttribPointer(2, 3, GL4.GL_FLOAT, false, 0, 0);

	    // Matrice de projection (identité ou zoom plus tard)
	    int transformLoc = gl.glGetUniformLocation(pointsShaderProgram, "u_transform");
	    gl.glUniformMatrix4fv(transformLoc, 1, false, projectionMatrix);

	    // Dessin des points
	    gl.glDrawArrays(GL4.GL_POINTS, 0, vertices.size());

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
    private void testInit() {
        // Initialisation du graphe avec le fichier à charger, la méthode de similitude et la méthode de détection de communautés
        String sample1 = "samples/iris.csv";
        String sample2 = "samples/predicancerNUadd9239.csv";
        initGraphCsv(sample2, GraphData.SimilitudeMode.CORRELATION, GraphData.NodeCommunity.LOUVAIN);

        setScreenSize(WIDTH, HEIGHT); // Taille de l'écran du graphe
        setBackgroundColor(0.0f, 0.0f, 0.0f); // Couleur de fond du graphe
        setUpscale(GRAPH_UPSCALE); // Facteur d'agrandissement pour le graphe
        setInitialNodeSize(15); // Taille initiale d'un sommet
        setDegreeScaleFactor(0.15); // Facteur d'agrandissement selon le degré d'un sommet
    }




    /**
     * Exemple d'actions sur le graphe (en attendant l'interface graphique)
     */
    private void testActions() {

        Timeline tl1 = new Timeline(
                new KeyFrame(Duration.seconds(15), e -> {
                    setMode(GraphData.GraphMode.SELECTION);
                    System.out.print("Switch to " + getMode());
                    System.out.println(" - Vous pouvez sélectionner et déplacer des sommets");
                })
        );
        tl1.setCycleCount(1);
        tl1.play();

		Timeline tl2 = new Timeline(
				new KeyFrame(Duration.seconds(20), e -> {
					setMode(GraphData.GraphMode.DELETE);
					System.out.print("Switch to " + getMode());
					System.out.println(" - Vous pouvez supprimer des sommets");
				})
		);
		tl2.setCycleCount(1);
		tl2.play();

        Timeline tl5 = new Timeline(
                new KeyFrame(Duration.seconds(30), e -> {
                    setMode(GraphData.GraphMode.RUN);
                    System.out.print("Back to " + getMode());
                    System.out.println(" - Exécution du graphe (en mouvement)");
                })
        );
        tl5.setCycleCount(1);
        tl5.play();

        /*Timeline tl6 = new Timeline(
                new KeyFrame(Duration.seconds(55), e -> {
                    setMiniumDegree(10);
                    System.out.println("Remove nodes with degree < 10");
                })
        );
        tl6.setCycleCount(1);
        tl6.play();

        Timeline tl7 = new Timeline(
                new KeyFrame(Duration.seconds(60), e -> {
                    setMiniumDegree(0);
                    System.out.println("Reset minimum degree restriction");
                })
        );
        tl7.setCycleCount(1);
        tl7.play();*/

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
    public double[][] initGraphCsv(String path, GraphData.SimilitudeMode mode, GraphData.NodeCommunity community) {
        if (path == null || path.isEmpty())
            throw new RuntimeException("initGraphCsv : Chemin du fichier non spécifié.");

        // Appeler startsProgram avant d'utiliser les données natives
        double[][] data = startsProgram(path);

        // Déterminer le mode de similitude à utiliser
        if (mode == null)
            throw new RuntimeException("initGraphCsv : Mode de similitude non spécifié.");
        int modeSimilitude = getModeSimilitude(mode);

        init_metadata = computeThreshold(modeSimilitude, 5);
        if (init_metadata == null)
            throw new RuntimeException("initGraphCsv : Une erreur est survenue lors du calcul des seuils.");

        double recommendedThreshold = init_metadata.getEdgeThreshold();
        double recommendedAntiThreshold = init_metadata.getAntiThreshold();

        System.out.println("Seuil recommandé pour les arêtes : " + recommendedThreshold);
        System.out.println("Seuil recommandé pour les anti-arêtes : " + recommendedAntiThreshold);

        // Valeurs imposées pour le moment (à modifier)
        //recommendedThreshold = 0.966;
        //recommendedAntiThreshold = 0.6;

        // Déterminer le mode de détection de communautés à utiliser
        if (community == null)
            throw new RuntimeException("initGraphCsv : Mode de détection de communautés non spécifié.");
        int modeCommunity = getModeCommunity(community);

        metadata = initiliazeGraph(modeCommunity, recommendedThreshold, recommendedAntiThreshold);

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
        //setDimension(WIDTH, HEIGHT); // TODO (toujours compliqué côté C)
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
		System.out.println("setInitialNodeSize : " + size);
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
            System.out.println("Changement de mode vers: " + mode);
            
            // Définir le nouveau mode
            isRunMode.set(mode == GraphData.GraphMode.RUN);
            isSelectionMode.set(mode == GraphData.GraphMode.SELECTION);
            isMoveMode.set(mode == GraphData.GraphMode.MOVE);
            isDeleteMode.set(mode == GraphData.GraphMode.DELETE);
            
            // Afficher un message selon le mode activé
            if (mode == GraphData.GraphMode.RUN) {
                System.out.println("Back to RUN - Exécution du graphe (en mouvement)");
            } else if (mode == GraphData.GraphMode.SELECTION) {
                System.out.println("Switch to SELECTION - Vous pouvez sélectionner et déplacer des sommets");
            } else if (mode == GraphData.GraphMode.MOVE) {
                System.out.println("Switch to MOVE - Vous pouvez vous déplacer dans le graphe");
            } else if (mode == GraphData.GraphMode.DELETE) {
                System.out.println("Switch to DELETE - Vous pouvez supprimer des sommets");
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
	 * @return l'histogramme // TODO
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
	public void setMiniumDegree(int degree) {
		if (degree < 0)
			throw new RuntimeException("setMiniumDegree : Degré minimum (" + degree + ") non valide.");
		minimumDegree.set(degree);
	}

	/**
	 * @param vertex_id Identifiant du sommet à supprimer
	 */
	public void removeVertex(int vertex_id) {
		// TODO
	}




    // -------------------------------------------------------------------------
    // Options supplémentaires
    // -------------------------------------------------------------------------

    /**
     * Exporte le graphe en image PNG
     * @param path Chemin de l'image PNG à exporter
     */
//    @Override
//    public void exportToPng(GL4 gl, String path) {
//        if (path == null || path.isEmpty()) {
//            throw new RuntimeException("exportToPng : Chemin du fichier non spécifié.");
//        }
//
//        File file = new File(path);
//        if (file.getParentFile() != null && !file.getParentFile().exists()) {
//            file.getParentFile().mkdirs();
//        }
//
//        if (!path.toLowerCase().endsWith(".png")) {
//            path += ".png";
//            file = new File(path);
//        }
//
//        int width = glWindow.getWidth();
//        int height = glWindow.getHeight();
//
//        ByteBuffer buffer = ByteBuffer.allocateDirect(width * height * 4);
//        buffer.order(ByteOrder.nativeOrder());
//
//        // Assurer que l'image est bien rendue avant lecture
//        gl.glReadBuffer(GL4.GL_BACK); // ← CORRECTION PRINCIPALE
//        gl.glFlush();
//        gl.glFinish();
//
//        gl.glReadPixels(0, 0, width, height, GL4.GL_RGBA, GL4.GL_UNSIGNED_BYTE, buffer);
//
//        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
//
//        buffer.rewind();
//        for (int y = height - 1; y >= 0; y--) {
//            for (int x = 0; x < width; x++) {
//                int r = buffer.get() & 0xFF;
//                int g = buffer.get() & 0xFF;
//                int b = buffer.get() & 0xFF;
//                int a = buffer.get() & 0xFF;
//
//                int pixel = (a << 24) | (r << 16) | (g << 8) | b;
//                image.setRGB(x, height - y - 1, pixel);
//            }
//        }
//
//        try {
//            ImageIO.write(image, "PNG", file);
//            System.out.println("Image exportée avec succès : " + path);
//        } catch (IOException e) {
//            throw new RuntimeException("Erreur lors de l'exportation de l'image : " + e.getMessage(), e);
//        }
//    }
    
    @Override
    public void exportToPng(GL4 gl, String path) {
        if (path == null || path.isEmpty()) {
            throw new RuntimeException("exportToPng : Chemin du fichier non spécifié.");
        }

        File file = new File(path);
        if (file.getParentFile() != null && !file.getParentFile().exists()) {
            file.getParentFile().mkdirs();
        }

        if (!path.toLowerCase().endsWith(".png")) {
            path += ".png";
            file = new File(path);
        }

        int width = glWindow.getWidth();
        int height = glWindow.getHeight();

        // Enregistrer l'état complet des ressources OpenGL
        int[] prevVAO = new int[1];
        gl.glGetIntegerv(GL4.GL_VERTEX_ARRAY_BINDING, prevVAO, 0);
        
        int[] prevArrayBuffer = new int[1];
        gl.glGetIntegerv(GL4.GL_ARRAY_BUFFER_BINDING, prevArrayBuffer, 0);
        
        int[] prevProgram = new int[1];
        gl.glGetIntegerv(GL4.GL_CURRENT_PROGRAM, prevProgram, 0);
        
        int[] prevFramebuffer = new int[1];
        gl.glGetIntegerv(GL4.GL_FRAMEBUFFER_BINDING, prevFramebuffer, 0);
        
        int[] prevViewport = new int[4];
        gl.glGetIntegerv(GL4.GL_VIEWPORT, prevViewport, 0);

        // Sauvegarder l'état des attributs de vertex
        boolean[] vertexAttribEnabled = new boolean[3];
        for (int i = 0; i < 3; i++) {
            int[] enabled = new int[1];
            gl.glGetVertexAttribiv(i, GL4.GL_VERTEX_ATTRIB_ARRAY_ENABLED, enabled, 0);
            vertexAttribEnabled[i] = (enabled[0] != 0);
        }

        // Créer un framebuffer et une texture pour le rendu hors écran
        int[] framebuffer = new int[1];
        int[] texture = new int[1];
        gl.glGenFramebuffers(1, framebuffer, 0);
        gl.glGenTextures(1, texture, 0);

        // Configurer la texture
        gl.glBindTexture(GL4.GL_TEXTURE_2D, texture[0]);
        gl.glTexImage2D(GL4.GL_TEXTURE_2D, 0, GL4.GL_RGBA, width, height, 0, GL4.GL_RGBA, GL4.GL_UNSIGNED_BYTE, null);
        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_MIN_FILTER, GL4.GL_LINEAR);
        gl.glTexParameteri(GL4.GL_TEXTURE_2D, GL4.GL_TEXTURE_MAG_FILTER, GL4.GL_LINEAR);
        
        // Attacher la texture au framebuffer
        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, framebuffer[0]);
        gl.glFramebufferTexture2D(GL4.GL_FRAMEBUFFER, GL4.GL_COLOR_ATTACHMENT0, GL4.GL_TEXTURE_2D, texture[0], 0);

        // Vérifier l'état du framebuffer
        if (gl.glCheckFramebufferStatus(GL4.GL_FRAMEBUFFER) != GL4.GL_FRAMEBUFFER_COMPLETE) {
            throw new RuntimeException("Framebuffer incomplet.");
        }

        // Configuration du viewport pour correspondre à la taille de la texture
        gl.glViewport(0, 0, width, height);
        
        // Effacer le buffer
        gl.glClear(GL4.GL_COLOR_BUFFER_BIT | GL4.GL_DEPTH_BUFFER_BIT);
        
        // Dessiner les cercles (utiliser un état temporaire)
        drawCircleForExport(gl, 0.0, 0.0, 1, 200);
        drawCircleForExport(gl, 0.5, 0.5, 1, 200);

        // Lire le contenu du framebuffer
        ByteBuffer buffer = ByteBuffer.allocateDirect(width * height * 4);
        buffer.order(ByteOrder.nativeOrder());
        gl.glReadPixels(0, 0, width, height, GL4.GL_RGBA, GL4.GL_UNSIGNED_BYTE, buffer);

        // Convertir en image
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        buffer.rewind();
        for (int y = height - 1; y >= 0; y--) {
            for (int x = 0; x < width; x++) {
                int r = buffer.get() & 0xFF;
                int g = buffer.get() & 0xFF;
                int b = buffer.get() & 0xFF;
                int a = buffer.get() & 0xFF;
                int pixel = (a << 24) | (r << 16) | (g << 8) | b;
                image.setRGB(x, height - y - 1, pixel);
            }
        }

        // Sauvegarder l'image
        try {
            ImageIO.write(image, "PNG", file);
            System.out.println("Image exportée avec succès : " + path);
        } catch (IOException e) {
            throw new RuntimeException("Erreur lors de l'exportation de l'image : " + e.getMessage(), e);
        }

        // Nettoyer les ressources temporaires
        gl.glDeleteFramebuffers(1, framebuffer, 0);
        gl.glDeleteTextures(1, texture, 0);

        // Restaurer l'état OpenGL précédent
        gl.glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, prevFramebuffer[0]);
        gl.glUseProgram(prevProgram[0]);
        gl.glBindVertexArray(prevVAO[0]);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, prevArrayBuffer[0]);
        
        // Restaurer les attributs de vertex
        for (int i = 0; i < 3; i++) {
            if (vertexAttribEnabled[i]) {
                gl.glEnableVertexAttribArray(i);
            } else {
                gl.glDisableVertexAttribArray(i);
            }
        }
        
        // Réinitialisation des buffers OpenGL
        reinitRender(gl);
    }

    // Méthode pour dessiner un cercle spécifiquement pour l'export
    private void drawCircleForExport(GL4 gl, double cx, double cy, double radius, int segments) {
        // Récupérer les dimensions actuelles pour calculer le ratio d'aspect
        int width = glWindow.getWidth();
        int height = glWindow.getHeight();
        float aspectRatio = (float)width / (float)height;
        
        // Shaders avec correction d'aspect ratio
        String vertexShaderSource = 
            "#version 400\n" +
            "layout(location = 0) in vec2 inPosition;\n" +
            "uniform float u_aspectRatio;\n" +  // Ajouter un uniform pour le ratio d'aspect
            "void main() {\n" +
            "    // Appliquer la correction d'aspect ratio\n" +
            "    vec2 correctedPos = vec2(inPosition.x, inPosition.y * u_aspectRatio);\n" +
            "    gl_Position = vec4(correctedPos, 0.0, 1.0);\n" +
            "}\n";
        
        String fragmentShaderSource =
				"""
						#version 400
						out vec4 FragColor;
						void main() {
						    FragColor = vec4(1.0, 0.0, 0.0, 1.0);
						}
						""";

        // Compiler les shaders
        int vertexShader = compileShader(gl, GL4.GL_VERTEX_SHADER, vertexShaderSource);
        int fragmentShader = compileShader(gl, GL4.GL_FRAGMENT_SHADER, fragmentShaderSource);

        // Créer le programme shader
        int exportShaderProgram = gl.glCreateProgram();
        gl.glAttachShader(exportShaderProgram, vertexShader);
        gl.glAttachShader(exportShaderProgram, fragmentShader);
        gl.glLinkProgram(exportShaderProgram);
        gl.glUseProgram(exportShaderProgram);
        
        // Définir le uniform pour le ratio d'aspect
        int aspectRatioLoc = gl.glGetUniformLocation(exportShaderProgram, "u_aspectRatio");
        gl.glUniform1f(aspectRatioLoc, 1.0f / aspectRatio); // Inverser pour corriger en Y

        // Données du cercle
        FloatBuffer circles = FloatBuffer.allocate((segments + 2) * 2);
        circles.put((float) cx);
        circles.put((float) cy);
        for (int i = 0; i <= segments; i++) {
            double angle = 2.0 * Math.PI * i / segments;
            double x = cx + radius * Math.cos(angle);
            double y = cy + radius * Math.sin(angle);
            circles.put((float) x);
            circles.put((float) y);
        }
        circles.flip();

        // Créer VAO et VBO temporaires
        int[] exportVAO = new int[1];
        gl.glGenVertexArrays(1, exportVAO, 0);
        gl.glBindVertexArray(exportVAO[0]);

        int[] exportVBO = new int[1];
        gl.glGenBuffers(1, exportVBO, 0);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, exportVBO[0]);
        gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) circles.limit() * Float.BYTES, circles, GL4.GL_STATIC_DRAW);

        // Configurer les attributs de vertex
        gl.glVertexAttribPointer(0, 2, GL4.GL_FLOAT, false, 0, 0);
        gl.glEnableVertexAttribArray(0);

        // Dessiner le cercle
        gl.glDrawArrays(GL4.GL_TRIANGLE_FAN, 0, segments + 2);

        // Nettoyer les ressources temporaires
        gl.glDeleteBuffers(1, exportVBO, 0);
        gl.glDeleteVertexArrays(1, exportVAO, 0);
        gl.glDeleteShader(vertexShader);
        gl.glDeleteShader(fragmentShader);
        gl.glDeleteProgram(exportShaderProgram);
    }

    // Méthode pour réinitialiser le rendu après l'export
    private void reinitRender(GL4 gl) {
        // Mettre à jour les données de rendu
        prepareVertexRenderData();
        prepareEdgeRenderData();
        
        // Recharger les données dans les buffers GPU
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexPoints.length * Float.BYTES, FloatBuffer.wrap(vertexPoints));
        
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexSizeBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexSizes.length * Float.BYTES, FloatBuffer.wrap(vertexSizes));
        
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, vertexColorBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) vertexColors.length * Float.BYTES, FloatBuffer.wrap(vertexColors));
        
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgePoints.length * Float.BYTES, FloatBuffer.wrap(edgePoints));
        
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeColorBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeColors.length * Float.BYTES, FloatBuffer.wrap(edgeColors));
        
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeSizeBuffer);
        gl.glBufferSubData(GL4.GL_ARRAY_BUFFER, 0, (long) edgeSizes.length * Float.BYTES, FloatBuffer.wrap(edgeSizes));
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
    // Buffer utilities
    // -------------------------------------------------------------------------
    
	private void createBuffers(GL4 gl) {
	    // Créer les buffers
	    int[] buffers = new int[3]; // Pour vertex, size, color
	    gl.glGenBuffers(3, buffers, 0);  // Génère 3 buffers à la fois

	    vertexBuffer = buffers[0];
	    vertexSizeBuffer = buffers[1];
	    vertexColorBuffer = buffers[2];

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
	}
	
	private void createEdgeBuffers(GL4 gl) {
	    int[] buffers = new int[4]; // Pour edge points, edge color, edge size, et edge visibility
	    gl.glGenBuffers(4, buffers, 0); // Génère 4 buffers

	    edgeBuffer = buffers[0];
	    edgeColorBuffer = buffers[1];
	    edgeSizeBuffer = buffers[2];
	    edgeVisibilityBuffer = buffers[3];

	    // Buffer pour les positions des arêtes
	    FloatBuffer edgeData = FloatBuffer.wrap(edgePoints);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgeData.limit() * Float.BYTES, edgeData, GL4.GL_STATIC_DRAW);

	    // Buffer pour les couleurs des arêtes
	    FloatBuffer edgeColorData = FloatBuffer.wrap(edgeColors);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeColorBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgeColorData.limit() * Float.BYTES, edgeColorData, GL4.GL_STATIC_DRAW);

	    // Buffer pour les tailles des arêtes
	    FloatBuffer edgeSizeData = FloatBuffer.wrap(edgeSizes);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeSizeBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgeSizeData.limit() * Float.BYTES, edgeSizeData, GL4.GL_STATIC_DRAW);

	    // Buffer pour la visibilité des arêtes
	    FloatBuffer edgeVisibilityData = FloatBuffer.wrap(edgeVisibility);
	    gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, edgeVisibilityBuffer);
	    gl.glBufferData(GL4.GL_ARRAY_BUFFER, (long) edgeVisibilityData.limit() * Float.BYTES, edgeVisibilityData, GL4.GL_STATIC_DRAW);
	}
	
	private void initializeArrays() {
		// Allouer avec une marge pour éviter des réallocations fréquentes
		int vertexCount = vertices.size();
		int edgeCount = edges.size();

		vertexPoints = new float[vertexCount * 2]; // x, y pour chaque sommet
		vertexSizes = new float[vertexCount]; // radius pour chaque sommet
		edgePoints = new float[edgeCount * 4]; // x1, y1, x2, y2 pour chaque arête
	    vertexColors = new float[vertexCount * 3]; // 3 composantes par sommet (R, G, B)
	    edgeSizes = new float[edgeCount]; // taille des arêtes
	    edgeColors = new float[edgeCount * 3]; // couleur des arêtes (R, G, B pour chaque arête)
	    edgeVisibility = new float[edgeCount * 2]; // 2 points par arête

        // Par défaut, toutes les arêtes sont visibles
        for (int i = 0; i < edges.size(); i++) {
            edgeVisibility[i * 2] = 1f;
            edgeVisibility[i * 2 + 1] = 1f;
        }
	    
	}

	/**
	 * Prépare les données de rendu pour les sommets
	 */
	private void prepareVertexRenderData() {
		Vertex currentVertex;
		Community currentCommunity;
	    for (int i = 0; i < vertices.size(); i++) {
	    	currentVertex = vertices.get(i);
	    	currentCommunity = vertices.get(i).getCommunity();
	    	
	    	// Mise à jour des buffers
	        vertexPoints[i * 2] = (float) currentVertex.getX();
	        vertexPoints[i * 2 + 1] = (float) currentVertex.getY();
	        vertexSizes[i] = (float) (currentVertex.getDiameter());
            vertexColors[i * 3] = currentCommunity.getR();
            vertexColors[i * 3 + 1] = currentCommunity.getG();
            vertexColors[i * 3 + 2] = currentCommunity.getB();
	    }
	}

	/**
	 * Prépare les données de rendu pour les arêtes
	 */
	private void prepareEdgeRenderData() {
		Edge currentEdge;
		Community startCommunity, endCommunity;
		Vertex startVertex, endVertex;
	    for (int i = 0; i < edges.size(); i++) {
	    	currentEdge = edges.get(i);
	        startVertex = currentEdge.getStart();
	        endVertex = currentEdge.getEnd();
	        startCommunity = startVertex.getCommunity();
	        endCommunity = endVertex.getCommunity();

	        // Mise à jour des buffers
            edgePoints[i * 4] = (float) startVertex.getX();
            edgePoints[i * 4 + 1] = (float) startVertex.getY();
            edgePoints[i * 4 + 2] = (float) endVertex.getX();
            edgePoints[i * 4 + 3] = (float) endVertex.getY();
			edgeSizes[i] = (float) (currentEdge.getWeight());
            edgeColors[i * 3] = (startCommunity.getR() + endCommunity.getR()) / 2;
            edgeColors[i * 3 + 1] = (startCommunity.getG() + endCommunity.getG()) / 2;
            edgeColors[i * 3 + 2] = (startCommunity.getB() + endCommunity.getB()) / 2;
            float visValue = (startVertex.isDeleted() || endVertex.isDeleted()) ? 0.0f : 1.0f;
            edgeVisibility[i * 2] = visValue;
            edgeVisibility[i * 2 + 1] = visValue;
	    }
	}
	



    // -------------------------------------------------------------------------
    // Shader utilities
    // -------------------------------------------------------------------------

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

	private String getShaderInfoLog(GL4 gl, int shader) {
	    IntBuffer logLength = IntBuffer.allocate(1);
	    gl.glGetShaderiv(shader, GL4.GL_INFO_LOG_LENGTH, logLength);
	    byte[] log = new byte[logLength.get(0)];
	    gl.glGetShaderInfoLog(shader, logLength.get(0), null, 0, log, 0);
	    return new String(log);
	}
	
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