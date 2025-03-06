#include <GL/glut.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <float.h>
#include <stdbool.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>

#include <stdatomic.h>
#include "../concurrent/Pool.h"

#ifdef _DEBUG_
    #include "../debug/debug_time.h"
#endif

#include "../../out/graph_Graph.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../lib/stb_image_write.h"

#define MAX_LINE_LENGTH 10000  // Longueur maximale d'une ligne dans le fichier
#define MAX_NODES 50000
#define MAX_EDGES 1000000
#define IMAGE_SIZE 15000
#define NUM_BINS 100  // Nombre de bins pour l'histogramme
#define MAX_SAMPLES 120000  // Maximum number of rows to sample
#define EPSILON 1e-12  // Pour éviter la division par 0
// Define the number of segments to approximate the circle (the higher the number, the smoother the circle)
#define NUM_SEGMENTS 50

// Structures de données pour représenter les arêtes, points, et clusters
typedef struct {
    int node1;
    int node2;
    double weight;
} Edge;



typedef struct {
    double x;
    double y;
} Point;

typedef struct {
    int *nodes;  // Tableau dynamique de noeuds dans le cluster
    int size;    // Nombre de noeuds actuels dans le cluster
    int capacity; // Capacité actuelle du tableau
} Cluster;

// Variables globales pour stocker les données de la simulation
char filename[1000];
int ll;
int modeA=0; //mode pour afficher des noeuds en fonction de classe
float bez=0.2;
double lambda=0.1;
int num_components = 0;
int saut=10;
int mode=0;
int num_nodes = 0;
int nbValeurs;
Edge edges[MAX_EDGES]; // Pour les arêtes normales
char *node_names[MAX_NODES]; // Array to store node names as strings      
Edge antiedges[MAX_EDGES];  // Pour les anti-arêtes
_Atomic int num_edges = 0;
_Atomic int num_antiedges = 0;
int S[MAX_NODES]={0};
Point positions[MAX_NODES];
Point velocities[MAX_NODES];
double centers[MAX_NODES][2];
int clusters[MAX_NODES];
Cluster *cluster_nodes = NULL;  // Tableau de clusters
double Lx = 300, Ly = 300;
int n_clusters;
double friction = 0.1;
double amortissement = 0.999;
double repulsion_coeff = 1;
double attraction_coeff = 100;
double coeff_antiarete = 100; // Facteur de répulsion des antiarêtes
double threshold;
double Max_movementOld=0;
int pause_simulation = 0;
int iteration=0;
double thresholdS = 1;
double thresholdA = 1;
int max_iterations = 5000;
int node_degrees[MAX_NODES];
float cluster_colors[MAX_NODES][3];
double **data = NULL;  // Stocker les données CSV
int num_rows = 0, num_columns = 0;
double epsilon = 0.1;
int centers_converged = 0;
int espacement = 1;
double seuilrep =0;
double correlation_threshold = 0.5; // Définir la valeur seuil pour l'affichage de la corrélation
    float initial_node_size=1.0;
    float degree_scale_factor=0.2;

struct Pool pool;

// Structures utilisées dans la méthode de Louvain
int communities[MAX_NODES]; // Stocke les communautés détectées par Louvain
typedef struct Neighbor {
    int node;
    double weight;
    struct Neighbor* next;
} Neighbor;

typedef struct {
    Neighbor* head;
} AdjacencyList;

typedef struct {
    int community;
    double total_weight;
    int component;  // Ajout du champ pour la composante connexe
} Community;

AdjacencyList adjacency_list[MAX_NODES];
Community node_community_map[MAX_NODES];
int num_communities=0;
int component_sizes[MAX_NODES]; // Tableau pour stocker la taille de chaque composante



// Prototypes de fonctions
void load_csv_data(const char *filename);
void sample_rows(int **sampled_rows, int *sampled_num_rows);
double calculate_mean_similitude(int num_samples,int);
void calculate_similitude_and_edges(double threshold, double antiseuil);
void random_point_in_plane(Point *p);
void normalize(Point *p);
void initialize_centers(void);
void update_positions(void);
void kmeans_iteration(Point *points, int num_points, int num_clusters, int *labels, double centers[][2], double Lx, double Ly);
void display(void);
void idle(void);
void assign_cluster_colors(void);
void calculate_node_degrees(void);
void save_image(const char *filename, int width, int height);
void save_image_opengl(int width, int height);
void draw_bezier_curve(Point p0, Point p1, Point control, int segments);
void init_clusters(int num_clusters);
void reinitialize_clusters(int new_num_clusters);
void free_clusters(void);
void add_node_to_cluster(int cluster_id, int node);
void clear_clusters(void);
void add_edge_to_adjacency_list(int node, int neighbor, double weight);
double calculate_gain_modularity(int node, int new_community, double total_graph_weight); 
int louvain_method();
void initialize_community_colors() ;
void parse_dot_file(const char *filename);
void render_text(float x, float y, const char *text);
int get_node_index(const char *name);
void translate_positions(double dx, double dy);
void compute_average_vectors();
int count_unique_communities(int *communities, int num_nodes);
double compute_norm(double *vector, int length);
void normalize_vector(double *vector, int length);
// Prototypes des fonctions de calcul des mesures de similarité
double cosine_similarity(int i, int j);
double euclidean_distance(int i, int j);
double L1_norm(int i, int j);
double Linf_norm(int i, int j);
double correlation_similarity(int i, int j);
int kbhit(void);
void find_connected_components();
void mark_component(int, int);
void apply_louvain_to_component(int);
int louvain_methodC();
void initialize_adjacency_list();
void lireColonneCSV(int *, int*);
void compute_ratio_S(int *);







// Fonction pour lire les valeurs de la première colonne d'un fichier CSV S[MAX_NODES]
void lireColonneCSV(int *S,int *nbValeurs) {
    char chemin[256];
    printf("Veuillez entrer le chemin du fichier CSV: ");
    // Use scanf with a width specifier to avoid buffer overflow
    scanf("%255s", chemin);  // Limiting the input to 255 characters
    FILE *fichier = fopen(chemin, "r");
    if (fichier == NULL) {
        perror("Erreur lors de l'ouverture du fichier");
        return;
    }

    char ligne[MAX_LINE_LENGTH];
    *nbValeurs = 0; // Initialisation du nombre de valeurs

    // Lecture de chaque ligne du fichier
    while (fgets(ligne, sizeof(ligne), fichier) != NULL) {
        // Séparer la première colonne en utilisant le point-virgule comme délimiteur
        char *token = strtok(ligne, ";");
        if (token != NULL) {
            // Vérifier si le tableau S n'est pas plein
            if (*nbValeurs < MAX_NODES) {
                // Convertir la chaîne en entier et stocker dans le tableau S
                S[*nbValeurs] = atoi(token);
                //printf("S[%d]=%d ",*nbValeurs,S[*nbValeurs]);
                (*nbValeurs)++;
            } else {
                printf("Nombre maximum de valeurs atteint.\n");
                break;
            }
        }
    }
    // Afficher les trois premières lignes
    printf("Les 5 premières lignes des données :\n");
    for (int i = 0; i < 5 && i < num_rows; i++) {
            printf("%d \n", S[i]);
    }
    fclose(fichier);
}


void draw_circle(float cx, float cy, float radius) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);  // Center of circle
    for (int i = 0; i <= NUM_SEGMENTS; i++) {
        float theta = 2.0f * 3.1415926f * (float)i / (float)NUM_SEGMENTS;  // Angle for each segment
        float x = radius * cosf(theta);  // Calculate x position
        float y = radius * sinf(theta);  // Calculate y position
        glVertex2f(cx + x, cy + y);      // Output vertex for the circle
    }
    glEnd();
}


void parse_dot_file(const char *filename) {
        printf("fichier %s bientôt ouvert",filename);
    fflush(stdout);
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    char line[10000];
    //int current_node = 0;
    printf("fichier %s ouvert",filename);
    fflush(stdout);
    while (fgets(line, sizeof(line), file)) {
        // Remove leading and trailing whitespace
        char *start = line;
        while (isspace(*start)) start++;
        char *end = start + strlen(start) - 1;
        while (end > start && isspace(*end)) end--;
        *(end + 1) = '\0';

        int node1, node2;
        double weight = 1.0; // Default weight if not provided

        // Check for directed edges (->)
        if (strstr(start, "->")) {
            // Extract nodes for directed edge
            if (sscanf(start, "%d -> %d", &node1, &node2) == 2) {
                // Update num_nodes
                if (node1 >= num_nodes) num_nodes = node1 + 1;
                if (node2 >= num_nodes) num_nodes = node2 + 1;

                // Add directed edge
                if (num_edges < MAX_EDGES) {
                    edges[num_edges].node1 = node1;
                    edges[num_edges].node2 = node2;
                    edges[num_edges].weight = weight;
                    num_edges++;
                } else {
                    fprintf(stderr, "Warning: Number of edges exceeds MAX_EDGES\n");
                }
            } else {
                fprintf(stderr, "Warning: Failed to parse directed edge line: %s\n", start);
            }
        }
        // Check for undirected edges (--)
        else if (strstr(start, "--")) {
            // Extract nodes for undirected edge
            if (sscanf(start, "%d -- %d", &node1, &node2) == 2) {
                // Update num_nodes
                if (node1 >= num_nodes) {num_nodes = node1 + 1;}
                if (node2 >= num_nodes) {num_nodes = node2 + 1;}

                // Add undirected edge (could be treated the same as directed in storage)
                if (num_edges < MAX_EDGES) {
                    edges[num_edges].node1 = node1;
                    edges[num_edges].node2 = node2;
                    edges[num_edges].weight = weight;
                    num_edges++;
                } else {
                    fprintf(stderr, "Warning: Number of edges exceeds MAX_EDGES\n");
                }
            } else {
                fprintf(stderr, "Warning: Failed to parse undirected edge line: %s\n", start);
            }
        }
    }

    fclose(file);
}


void initialize_adjacency_list() {
    for (int i = 0; i < MAX_NODES; i++) {
        adjacency_list[i].head = NULL;  // Initialiser chaque tête de liste à NULL
    }
}


// Fonction principale qui applique la méthode de Louvain à toutes les composantes connexes
int louvain_methodC() {

    
       initialize_adjacency_list();
    // Ajouter des arêtes (par exemple, à partir de la structure `edges[]`)
    for (int i = 0; i < num_edges; i++) {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;
        double weight = edges[i].weight;

        add_edge_to_adjacency_list(node1, node2, weight);
    }
    // Étape 1 : Trouver les composantes connexes
    find_connected_components();

    // Étape 2 : Appliquer la méthode de Louvain à chaque composante
    for (int component = 0; component < num_components; component++) {
        apply_louvain_to_component(component);
    }

    // Compter le nombre total de communautés dans le graphe
    int total_communities = count_unique_communities(communities, num_nodes);
    printf("Total number of communities detected: %d\n", total_communities);
    return total_communities;
}



// Fonction principale qui applique la méthode de Louvain à une composante
void apply_louvain_to_component(int component) {
    double total_graph_weight = 0.0;

    // Initialisation des listes d'adjacence et des communautés pour la composante donnée
    for (int i = 0; i < num_edges; i++) {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;
        double weight = edges[i].weight;

        if (node_community_map[node1].component != component || node_community_map[node2].component != component) {
            continue;  // Ignorer les arêtes qui ne font pas partie de la composante actuelle
        }

        add_edge_to_adjacency_list(node1, node2, weight);
        add_edge_to_adjacency_list(node2, node1, weight);

        node_community_map[node1].community = node1;
        node_community_map[node1].total_weight += weight;
        node_community_map[node2].community = node2;
        node_community_map[node2].total_weight += weight;

        total_graph_weight += 2 * weight;
    }

    int improvement = 1;
    while (improvement) {
        improvement = 0;

        for (int node = 0; node < num_nodes; node++) {
            if (node_community_map[node].component != component) {
                continue;  // Ignorer les nœuds qui ne font pas partie de la composante actuelle
            }

            int current_community = node_community_map[node].community;
            double max_delta_modularity = 0.0;
            int best_community = current_community;

            // Calcul du gain de modularité pour le déplacement du nœud
            Neighbor* neighbor = adjacency_list[node].head;
            while (neighbor != NULL) {
                int neighbor_community = node_community_map[neighbor->node].community;
                if (neighbor_community == current_community) {
                    neighbor = neighbor->next;
                    continue;
                }

                double delta_modularity = calculate_gain_modularity(node, neighbor_community, total_graph_weight);
                if (delta_modularity > max_delta_modularity) {
                    max_delta_modularity = delta_modularity;
                    best_community = neighbor_community;
                }
                neighbor = neighbor->next;
            }

            // Mise à jour de la communauté si une meilleure est trouvée
            if (best_community != current_community) {
                node_community_map[node].community = best_community;
                improvement = 1;
            }
        }
    }

    // Mise à jour des communautés détectées pour cette composante
    for (int i = 0; i < num_nodes; i++) {
        if (node_community_map[i].component == component) {
            communities[i] = node_community_map[i].community;
        }
    }

    // Afficher le nombre de communautés uniques pour cette composante
    //int num_communities = count_unique_communities(communities, num_nodes);
    //printf("Number of communities detected in component %d: %d\n", component, num_communities);
}



// Fonction pour détecter les composantes connexes du graphe
void find_connected_components() {
    // Initialiser toutes les composantes à -1 (non visitées)
    for (int i = 0; i < num_nodes; i++) {
        node_community_map[i].component = -1;
    }
    // Initialiser les tailles des composantes à 0
    for (int i = 0; i < MAX_NODES; i++) {
        component_sizes[i] = 0;
    }

    num_components = 0;  // Initialisation du nombre de composantes

    // Parcourir tous les nœuds du graphe
    for (int i = 0; i < num_nodes; i++) {
        if (node_community_map[i].component == -1) {
            // Si le nœud n'a pas été visité, on lance DFS pour marquer tous les nœuds de la composante
            mark_component(i, num_components);
            num_components++;  // Incrémenter le nombre de composantes après en avoir trouvé une
        }
    }

    // Afficher le nombre de composantes connexes et leur taille
    printf("Number of connected components: %d\n", num_components);
    //for (int component = 0; component < num_components; component++) {
    //    printf("Component %d has %d nodes.\n", component, component_sizes[component]);
    //}
}


// Fonction pour marquer une composante (DFS ou BFS)
// Fonction DFS pour marquer les nœuds d'une composante
void mark_component(int node, int component) {
    // Marquer le nœud comme appartenant à la composante courante
    node_community_map[node].component = component;
    component_sizes[component]++;  // Incrémenter la taille de la composante

    // Parcourir tous les voisins du nœud
    Neighbor* neighbor = adjacency_list[node].head;
    while (neighbor != NULL) {
        int neighbor_node = neighbor->node;
        // Si le voisin n'a pas encore été visité (composante == -1), on le marque
        if (node_community_map[neighbor_node].component == -1) {
            mark_component(neighbor_node, component);  // Appel récursif pour le voisin
        }
        neighbor = neighbor->next;
    }
}


int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt); // Sauvegarde des paramètres du terminal
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Désactive la mise en buffer et l'écho
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Applique les nouveaux paramètres
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // Met le mode non bloquant

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restaure les anciens paramètres
    fcntl(STDIN_FILENO, F_SETFL, oldf); // Remet le mode bloquant

    if (ch != EOF) {
        ungetc(ch, stdin); // Remet le caractère dans le flux d'entrée
        return 1;
    }

    return 0;
}



// Fonction pour calculer la divergence KL entre deux vecteurs normalisés
double KL_divergence(int i, int j) {
    double kl_div = 0.0;
    double p[num_columns];
    double q[num_columns];

    // Copier les lignes i et j dans des vecteurs locaux pour les normaliser
    for (int k = 0; k < num_columns; k++) {
        p[k] = data[i][k];
        q[k] = data[j][k];
    }

    // Normaliser les deux vecteurs
    normalize_vector(p, num_columns);
    normalize_vector(q, num_columns);

    // Calcul de la divergence KL
    for (int k = 0; k < num_columns; k++) {
        if (p[k] > EPSILON && q[k] > EPSILON) {
            kl_div += p[k] * log(p[k] / q[k]);
        }
    }
    
    return 1/(1+kl_div);
}

// Calcul de la distance cosinus
double cosine_similarity(int i, int j) {
    double num = 0.0, den_i = 0.0, den_j = 0.0;
    for (int k = 0; k < num_columns; k++) {
        num += data[i][k] * data[j][k];
        den_i += data[i][k] * data[i][k];
        den_j += data[j][k] * data[j][k];
    }
    return num / (sqrt(den_i) * sqrt(den_j));
}

// Calcul de la distance euclidienne
double euclidean_distance(int i, int j) {
    double sum = 0.0;
    for (int k = 0; k < num_columns; k++) {
        double diff = data[i][k] - data[j][k];
        sum += diff * diff;
    }
    return 1/(1+sum);
}

// Calcul de la norme L1 (somme des distances absolues)
double L1_norm(int i, int j) {
    double sum = 0.0;
    for (int k = 0; k < num_columns; k++) {
        sum += fabs(data[i][k] - data[j][k]);
    }
    return 1/(1+sum);
}

// Calcul de la norme Linf (distance maximale entre les composantes des vecteurs)
double Linf_norm(int i, int j) {
    double max_diff = 0.0;
    for (int k = 0; k < num_columns; k++) {
        double diff = fabs(data[i][k] - data[j][k]);
        if (diff > max_diff) {
            max_diff = diff;
        }
    }
    return 1/(1+max_diff);
}

// Calcul de la corrélation entre deux vecteurs
double correlation_similarity(int i, int j) {
    double mean_i = 0.0, mean_j = 0.0;
    for (int k = 0; k < num_columns; k++) {
        mean_i += data[i][k];
        mean_j += data[j][k];
    }
    mean_i /= num_columns;
    mean_j /= num_columns;

    double num = 0.0, den_i = 0.0, den_j = 0.0;
    for (int k = 0; k < num_columns; k++) {
        double di = data[i][k] - mean_i;
        double dj = data[j][k] - mean_j;
        num += di * dj;
        den_i += di * di;
        den_j += dj * dj;
    }

    return (num / sqrt(den_i * den_j));
}


// Function to compute the norm of a vector
double compute_norm(double *vector, int length) {
    double norm = 0.0;
    for (int i = 0; i < length; i++) {
        norm += vector[i] * vector[i];
    }
    return sqrt(norm);
}

// Function to normalize a vector
void normalize_vector(double *vector, int length) {
    double norm = compute_norm(vector, length);
    if (norm > 0) {
        for (int i = 0; i < length; i++) {
            vector[i] /= norm;
        }
    }
}

// Fonction pour compter et retourner le nombre de communautés uniques
int count_unique_communities(int *communities, int num_nodes) {
    int *unique_communities = (int *)malloc(num_nodes * sizeof(int));
    int unique_count = 0;

    for (int i = 0; i < num_nodes; i++) {
        int community = communities[i];
        int is_new = 1;
        
        // Vérifier si la communauté a déjà été comptée
        for (int j = 0; j < unique_count; j++) {
            if (unique_communities[j] == community) {
                is_new = 0;
                break;
            }
        }

        // Si la communauté est nouvelle, l'ajouter à la liste des communautés uniques
        if (is_new) {
            unique_communities[unique_count] = community;
            unique_count++;
        }
    }

    free(unique_communities);  // Libérer la mémoire après l'utilisation
    return unique_count;
}

// Function to compute the ratio of S[i] = 1 in each community and display the results
void compute_ratio_S(int *S) {
    // Check if the number of clusters is valid
    if (num_communities <= 0) {
        printf("Error: num_communities is not correctly initialized.\n");
        return;
    }

    // Array to store the size of each community
    int *community_sizes = (int *)calloc(num_nodes, sizeof(int));
    if (community_sizes == NULL) {
        printf("Error: Memory allocation failed for community_sizes.\n");
        return;
    }

    // Array to count occurrences of S[i] = 1 in each community
    int *community_s1_counts = (int *)calloc(num_nodes, sizeof(int));
    if (community_s1_counts == NULL) {
        printf("Error: Memory allocation failed for community_s1_counts.\n");
        free(community_sizes);
        return;
    }
        //printf("TEST S{3}=%d, %d communities \n",S[3],num_rows);
       //fflush(stdout);
    // Traverse through each node and update community sizes and S[i] = 1 counts
    for (int i = 0; i < num_rows; i++) {
        int community = communities[i];
        //printf("TEST S{3}=%d, %d communities %d \n",S[3],i,communities[i]);
       //fflush(stdout);
        // Check that the community index is within the valid range
        if (community < 0 || community >= num_nodes) {
            printf("Error: Invalid community index %d for node %d.\n", community, i);
            continue;
        }

        community_sizes[community]++;
        if (S[i] == 1) {
            community_s1_counts[community]++;
        }
    }

    // Display the ratio of S[i] = 1 for each community with more than 1% of nodes
    printf("Community Ratios (From Largest to Smallest) containing more than 0.5 percent of the nodes:\n");

    for (int comm = 0; comm < num_nodes; comm++) {
        if (community_sizes[comm] > num_nodes / 200) {
            double ratio_s1 = (double)community_s1_counts[comm] / community_sizes[comm];

            printf("Community %d: Size = %d, Ratio of 1 = %.10f\n", 
                   comm, community_sizes[comm], ratio_s1);
                   fflush(stdout);
        }
    }

    // Free allocated memory
    free(community_sizes);
    free(community_s1_counts);
}


// Function to compute the average vector for each community and display the results
void compute_average_vectors() {
    // Check if the number of clusters is valid
    if (num_communities <= 0) {
        printf("Error: num_communities is not correctly initialized.\n");
        return;
    }

    // Array to store the size of each community
    int *community_sizes = (int *)calloc(num_nodes, sizeof(int));  
    if (community_sizes == NULL) {
        printf("Error: Memory allocation failed for community_sizes.\n");
        return;
    }

    // Array to store the sum of vectors in each community
    double **community_sums = (double **)malloc(num_nodes * sizeof(double *));  
    if (community_sums == NULL) {
        printf("Error: Memory allocation failed for community_sums.\n");
        free(community_sizes);
        return;
    }

    // Initialize arrays for summing the vectors
    for (int i = 0; i < num_nodes; i++) {
        community_sums[i] = (double *)calloc(num_columns, sizeof(double));
        if (community_sums[i] == NULL) {
            printf("Error: Memory allocation failed for community_sums[%d].\n", i);
            // Free previously allocated memory in case of error
            for (int j = 0; j < i; j++) {
                free(community_sums[j]);
            }
            free(community_sums);
            free(community_sizes);
            return;
        }
    }

    // Traverse through each node and add the data to the corresponding community
    for (int i = 0; i < num_rows; i++) {
        int community = communities[i];

        // Check that the community index is within the valid range
        if (community < 0 || community >= num_nodes) {
            printf("Error: Invalid community index %d for node %d.\n", community, i);
            continue;
        }

        community_sizes[community]++;
        for (int j = 0; j < num_columns; j++) {
            community_sums[community][j] += data[i][j];
        }
    }

    // Compute and display the average vector for each community
    printf("Community Averages (From Largest to Smallest) containing more than 0.5 pourcent of the nodes :\n");

    for (int comm = 0; comm < num_nodes; comm++) {
        if (community_sizes[comm] > num_nodes/200) {
            // Compute the average vector for this community
            for (int j = 0; j < num_columns; j++) {
                community_sums[comm][j] /= community_sizes[comm];  // Calculate the average
            }
            
            // Normalize the average vector
            //normalize_vector(community_sums[comm], num_columns);

            // Display the normalized average vector
            printf("Community %d: Size = %d, Normalized Average Vector = [", comm, community_sizes[comm]);
            for (int j = 0; j < num_columns; j++) {
                printf("%.10f", community_sums[comm][j]);
                if (j < num_columns - 1) {
                    printf(", ");
                }
            }
            printf("]\n");
        }
    }

    // Free allocated memory
    for (int i = 0; i < num_nodes; i++) {
        free(community_sums[i]);
    }
    free(community_sums);
    free(community_sizes);
}



void translate_positions(double dx, double dy) {
    for (int i = 0; i < num_nodes; i++) {
        positions[i].x += dx;
        positions[i].y += dy;
        while (positions[i].x < -Lx/2) positions[i].x += Lx;
            while (positions[i].x > Lx/2) positions[i].x -= Lx;
            while (positions[i].y < -Ly/2) positions[i].y += Ly;
            while (positions[i].y > Ly/2) positions[i].y -= Ly;
    }
}



    // Initialisez les couleurs pour chaque communauté (ici, on suppose un maximum de MAX_NODES communautés)
void initialize_community_colors() {

for (int i = 0; i < num_nodes; i++) {
    float min_color_value = 0.2;  // seuil minimal pour éviter les couleurs sombres
    cluster_colors[i][0] = min_color_value + (float)rand() / RAND_MAX * (1.0 - min_color_value); // Couleur R
    cluster_colors[i][1] = min_color_value + (float)rand() / RAND_MAX * (1.0 - min_color_value); // Couleur G
    cluster_colors[i][2] = min_color_value + (float)rand() / RAND_MAX * (1.0 - min_color_value); // Couleur B
}
}


// Fonction pour calculer le gain de modularité pour le déplacement d'un nœud
double calculate_gain_modularity(int node, int new_community, double total_graph_weight) {
    double current_modularity = 0.0;
    double new_modularity = 0.0;
    
    int current_community = node_community_map[node].community;

    // Calcul de la contribution des arêtes internes et externes pour la communauté actuelle
    Neighbor* neighbor = adjacency_list[node].head;
    while (neighbor != NULL) {
        int neighbor_community = node_community_map[neighbor->node].community;
        if (neighbor_community == current_community) {
            current_modularity += neighbor->weight;
        } else if (neighbor_community == new_community) {
            new_modularity += neighbor->weight;
        }
        neighbor = neighbor->next;
    }

    // Calcul de la modularité avant et après le déplacement
    double current_weight = node_community_map[node].total_weight;
    double neighbor_weight = node_community_map[new_community].total_weight;
    double delta_modularity = (new_modularity - current_modularity) - 
                              (current_weight * neighbor_weight) / total_graph_weight;
    
    return delta_modularity;
}


// Fonction pour ajouter un voisin dans la liste d'adjacence
void add_edge_to_adjacency_list(int node, int neighbor, double weight) {
    Neighbor* new_neighbor = (Neighbor*)malloc(sizeof(Neighbor));
    new_neighbor->node = neighbor;
    new_neighbor->weight = weight;
    new_neighbor->next = adjacency_list[node].head;
    adjacency_list[node].head = new_neighbor;
}

int louvain_method() {
    double total_graph_weight = 0.0;
       initialize_adjacency_list();
    // Initialisation des listes d'adjacence et des communautés
    for (int i = 0; i < num_edges; i++) {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;
        double weight = edges[i].weight;

        add_edge_to_adjacency_list(node1, node2, weight);
        add_edge_to_adjacency_list(node2, node1, weight);

        node_community_map[node1].community = node1;
        node_community_map[node1].total_weight += weight;
        node_community_map[node2].community = node2;
        node_community_map[node2].total_weight += weight;

        total_graph_weight += 2 * weight;
    }

       // Après avoir construit la liste d'adjacence, détecter les composantes connexes
    find_connected_components();    

    int improvement = 1;
    while (improvement) {
        improvement = 0;

        for (int node = 0; node < num_nodes; node++) {
            int current_community = node_community_map[node].community;
            double max_delta_modularity = 0.0;
            int best_community = current_community;

            // Calcul du gain de modularité pour le déplacement du nœud
            Neighbor* neighbor = adjacency_list[node].head;
            while (neighbor != NULL) {
                int neighbor_community = node_community_map[neighbor->node].community;
                if (neighbor_community == current_community) {
                    neighbor = neighbor->next;
                    continue;
                }

                double delta_modularity = calculate_gain_modularity(node, neighbor_community, total_graph_weight);
                if (delta_modularity > max_delta_modularity) {
                    max_delta_modularity = delta_modularity;
                    best_community = neighbor_community;
                }
                neighbor = neighbor->next;
            }

            // Mise à jour de la communauté si une meilleure est trouvée
            if (best_community != current_community) {
                node_community_map[node].community = best_community;
                improvement = 1;
            }
        }
    }

    // Mise à jour des communautés détectées
    for (int i = 0; i < num_nodes; i++) {
        communities[i] = node_community_map[i].community;
    }

    // Display the number of unique communities at the end of the Louvain method
    int num_communities = count_unique_communities(communities, num_nodes);
    printf("Number of communities detected: %d\n", num_communities);
    return num_communities;
}



// Assigner des couleurs de clusters aux noeuds
void assign_colors_to_nodes() {
    for (int i = 0; i < num_nodes; i++) {
        int cluster = clusters[i];
        glColor3f(cluster_colors[cluster][0], cluster_colors[cluster][1], cluster_colors[cluster][2]);
    }
}


// Fonction pour vérifier la connectivité interne d'une communauté
int is_strongly_connected(int community_id) {
    // Parcours en profondeur ou en largeur pour vérifier si tous les nœuds de la communauté sont connectés
    int visited[MAX_NODES] = {0};
    int stack[MAX_NODES], top = -1;
    int start_node = -1;

    // Trouver un nœud de départ dans la communauté
    for (int i = 0; i < num_nodes; i++) {
        if (node_community_map[i].community == community_id) {
            start_node = i;
            break;
        }
    }

    if (start_node == -1) return 0; // Aucun nœud trouvé

    // Initialiser la pile avec le nœud de départ
    stack[++top] = start_node;
    visited[start_node] = 1;

    while (top != -1) {
        int current_node = stack[top--];
        Neighbor* neighbor = adjacency_list[current_node].head;
        while (neighbor != NULL) {
            if (node_community_map[neighbor->node].community == community_id && !visited[neighbor->node]) {
                visited[neighbor->node] = 1;
                stack[++top] = neighbor->node;
            }
            neighbor = neighbor->next;
        }
    }

    // Vérifier si tous les nœuds de la communauté ont été visités
    for (int i = 0; i < num_nodes; i++) {
        if (node_community_map[i].community == community_id && !visited[i]) {
            return 0; // La communauté n'est pas connectée
        }
    }

    return 1; // La communauté est connectée
}

// Fonction pour calculer le gain de modularité pour un nœud et une nouvelle communauté avec connectivité garantie
double calculate_gain_modularity_leiden(int node, int new_community, double total_graph_weight) {
    double current_modularity = 0.0;
    double new_modularity = 0.0;
    
    int current_community = node_community_map[node].community;

    // Calcul de la contribution des arêtes internes et externes pour la communauté actuelle
    Neighbor* neighbor = adjacency_list[node].head;
    while (neighbor != NULL) {
        int neighbor_community = node_community_map[neighbor->node].community;
        if (neighbor_community == current_community) {
            current_modularity += neighbor->weight;
        } else if (neighbor_community == new_community) {
            new_modularity += neighbor->weight;
        }
        neighbor = neighbor->next;
    }

    // Calcul de la modularité avant et après le déplacement
    double current_weight = node_community_map[node].total_weight;
    double neighbor_weight = node_community_map[new_community].total_weight;
    double delta_modularity = (new_modularity - current_modularity) - 
                              (current_weight * neighbor_weight) / total_graph_weight;
    
    // Vérifier si la nouvelle communauté reste connectée après l'ajout du nœud
    if (!is_strongly_connected(new_community)) {
        return -1; // Si la communauté n'est plus connectée, ne pas effectuer le déplacement
    }

    return delta_modularity;
}


//CPM
double calculate_gain_modularity_cpm(int node, int new_community, double resolution_parameter) {
    double current_internal_edges = 0.0;
    double new_internal_edges = 0.0;
    
    int current_community = node_community_map[node].community;
    int degree_node = 0;  // Le degré du nœud (somme des poids des arêtes)

    // Calcul des contributions des arêtes internes à la communauté actuelle et à la nouvelle
    Neighbor* neighbor = adjacency_list[node].head;
    while (neighbor != NULL) {
        int neighbor_community = node_community_map[neighbor->node].community;
        if (neighbor_community == current_community) {
            current_internal_edges += neighbor->weight;
        } else if (neighbor_community == new_community) {
            new_internal_edges += neighbor->weight;
        }
        degree_node += neighbor->weight;
        neighbor = neighbor->next;
    }

    // Calcul des tailles des communautés avant et après le déplacement
    double current_community_size = node_community_map[current_community].total_weight;
    double new_community_size = node_community_map[new_community].total_weight;

    // Calcul du gain de modularité en utilisant la formule CPM
    double delta_modularity = new_internal_edges - current_internal_edges
                              - resolution_parameter * (degree_node * (new_community_size - current_community_size));
    
    return delta_modularity;
}


// Algorithme de Leiden
int leiden_method() {
    double total_graph_weight = 0.0;

    // Initialisation des listes d'adjacence et des communautés
    for (int i = 0; i < num_edges; i++) {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;
        double weight = edges[i].weight;

        add_edge_to_adjacency_list(node1, node2, weight);
        add_edge_to_adjacency_list(node2, node1, weight);

        node_community_map[node1].community = node1;
        node_community_map[node1].total_weight += weight;
        node_community_map[node2].community = node2;
        node_community_map[node2].total_weight += weight;

        total_graph_weight += 2 * weight;
    }

    int improvement = 1;
    while (improvement) {
        improvement = 0;

        for (int node = 0; node < num_nodes; node++) {
            int current_community = node_community_map[node].community;
            double max_delta_modularity = 0.0;
            int best_community = current_community;

            // Calcul du gain de modularité pour le déplacement du nœud avec Leiden
            Neighbor* neighbor = adjacency_list[node].head;
            while (neighbor != NULL) {
                int neighbor_community = node_community_map[neighbor->node].community;
                if (neighbor_community == current_community) {
                    neighbor = neighbor->next;
                    continue;
                }

                double delta_modularity = calculate_gain_modularity_leiden(node, neighbor_community, total_graph_weight);
                if (delta_modularity > max_delta_modularity) {
                    max_delta_modularity = delta_modularity;
                    best_community = neighbor_community;
                }
                neighbor = neighbor->next;
            }

            // Mise à jour de la communauté si une meilleure est trouvée
            if (best_community != current_community) {
                node_community_map[node].community = best_community;
                improvement = 1;
            }
        }
    }

    // Mise à jour des communautés détectées
    for (int i = 0; i < num_nodes; i++) {
        communities[i] = node_community_map[i].community;
    }

    // Afficher le nombre de communautés uniques à la fin de l'algorithme de Leiden
    int num_communities = count_unique_communities(communities, num_nodes);
    printf("Number of communities detected: %d\n", num_communities);
    return num_communities;
}


// Algorithme de Leiden CPM
int leiden_method_CPM() {
    double total_graph_weight = 0.0;

    // Initialisation des listes d'adjacence et des communautés
    for (int i = 0; i < num_edges; i++) {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;
        double weight = edges[i].weight;

        add_edge_to_adjacency_list(node1, node2, weight);
        add_edge_to_adjacency_list(node2, node1, weight);

        node_community_map[node1].community = node1;
        node_community_map[node1].total_weight += weight;
        node_community_map[node2].community = node2;
        node_community_map[node2].total_weight += weight;

        total_graph_weight += 2 * weight;
    }

    int improvement = 1;
    while (improvement) {
        improvement = 0;

        for (int node = 0; node < num_nodes; node++) {
            int current_community = node_community_map[node].community;
            double max_delta_modularity = 0.0;
            int best_community = current_community;

            // Calcul du gain de modularité pour le déplacement du nœud avec Leiden
            Neighbor* neighbor = adjacency_list[node].head;
            while (neighbor != NULL) {
                int neighbor_community = node_community_map[neighbor->node].community;
                if (neighbor_community == current_community) {
                    neighbor = neighbor->next;
                    continue;
                }

                double delta_modularity = calculate_gain_modularity_cpm(node, neighbor_community, lambda);
                if (delta_modularity > max_delta_modularity) {
                    max_delta_modularity = delta_modularity;
                    best_community = neighbor_community;
                }
                neighbor = neighbor->next;
            }

            // Mise à jour de la communauté si une meilleure est trouvée
            if (best_community != current_community) {
                node_community_map[node].community = best_community;
                improvement = 1;
            }
        }
    }

    // Mise à jour des communautés détectées
    for (int i = 0; i < num_nodes; i++) {
        communities[i] = node_community_map[i].community;
    }

    // Afficher le nombre de communautés uniques à la fin de l'algorithme de Leiden
    int num_communities = count_unique_communities(communities, num_nodes);
    printf("Number of communities detected: %d\n", num_communities);
    return num_communities;
}


// Gérer les clics de souris pour mettre en pause la simulation et accéder aux options
void mouse_click(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        pause_simulation = !pause_simulation;  // Toggle pause on left mouse click
    }
}

// Réinitialiser les clusters si le nombre de clusters change
void reinitialize_clusters(int new_num_clusters) {
    if (new_num_clusters != n_clusters) {
        free_clusters();
        init_clusters(new_num_clusters);
    } else {
        clear_clusters();
    }
    initialize_centers();
    assign_cluster_colors();
    n_clusters = new_num_clusters;
}

// Initialiser les clusters avec des tableaux dynamiques
void init_clusters(int num_clusters) {
    int estimated_capacity = (num_nodes / num_clusters) + 1;
    cluster_nodes = (Cluster *)malloc(num_clusters * sizeof(Cluster));
    for (int i = 0; i < num_clusters; i++) {
        cluster_nodes[i].nodes = (int *)malloc(estimated_capacity * sizeof(int));
        cluster_nodes[i].size = 0;
        cluster_nodes[i].capacity = estimated_capacity;
    }
}

// Libérer la mémoire allouée pour les clusters
void free_clusters() {
    if (cluster_nodes != NULL) {
        for (int i = 0; i < n_clusters; i++) {
            if (cluster_nodes[i].nodes != NULL) {
                free(cluster_nodes[i].nodes);
                cluster_nodes[i].nodes = NULL;
            }
        }
        free(cluster_nodes);
        cluster_nodes = NULL;
    }
}

// Ajouter un noeud à un cluster, redimensionner si nécessaire
void add_node_to_cluster(int cluster_id, int node) {
    Cluster *cluster = &cluster_nodes[cluster_id];
    if (cluster->size == cluster->capacity) {
        cluster->capacity *= 2;
        cluster->nodes = (int *)realloc(cluster->nodes, cluster->capacity * sizeof(int));
    }
    cluster->nodes[cluster->size++] = node;
}

// Vider les clusters
void clear_clusters() {
    for (int i = 0; i < n_clusters; i++) {
        cluster_nodes[i].size = 0;
    }
}

// Fonction pour échantillonner un sous-ensemble de lignes
void sample_rows(int **sampled_rows, int *sampled_num_rows) {
    int *rows = malloc(num_rows * sizeof(int));
    for (int i = 0; i < num_rows; i++) {
        rows[i] = i;
    }

    // Mélanger les lignes
    for (int i = num_rows - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int temp = rows[i];
        rows[i] = rows[j];
        rows[j] = temp;
    }

    *sampled_num_rows = (num_rows > MAX_SAMPLES) ? MAX_SAMPLES : num_rows;
    *sampled_rows = malloc(*sampled_num_rows * sizeof(int));
    for (int i = 0; i < *sampled_num_rows; i++) {
        (*sampled_rows)[i] = rows[i];
    }
    free(rows);
}

// Calculer les degrés de chaque noeud
void calculate_node_degrees(void) {
    for (int i = 0; i < num_nodes; i++) {
        node_degrees[i] = 0;
    }
    for (int i = 0; i < num_edges; i++) {
        node_degrees[edges[i].node1]++;
        node_degrees[edges[i].node2]++;
    }
}

// Dessiner une courbe de Bézier entre deux points
void draw_bezier_curve(Point p0, Point p1, Point control, int segments) {
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= segments; i++) {
        float t = (float)i / (float)segments;
        float u = 1 - t;
        float x = u * u * p0.x + 2 * u * t * control.x + t * t * p1.x;
        float y = u * u * p0.y + 2 * u * t * control.y + t * t * p1.y;
        glVertex2f(x, y);
    }
    glEnd();
}

// Pour éviter les warnings à la compilation
void* glGenFramebuffers(int arg1, void* arg2);
void* glBindFramebuffer(int arg1, unsigned int arg2);
void* glGenRenderbuffers(int arg1, void* arg2);
void* glBindRenderbuffer(int arg1, unsigned int arg2);
void* glRenderbufferStorage(int arg1, int arg2, int arg3, int arg4);
void* glFramebufferRenderbuffer(int arg1, int arg2, int arg3, unsigned int arg4);
int glCheckFramebufferStatus(int arg1);
void* glDeleteRenderbuffers(int arg1, void* arg2);
void* glDeleteFramebuffers(int arg1, void* arg2);

// Sauvegarder la scène OpenGL comme image
void save_image_opengl(int width, int height) {
    GLuint fbo, renderBuffer, depthBuffer;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenRenderbuffers(1, &renderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, renderBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, renderBuffer);

    glGenRenderbuffers(1, &depthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        printf("Failed to create framebuffer!\n");
        return;
    }

    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-Lx / 2, Lx / 2, -Ly / 2, Ly / 2);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Dessiner les noeuds
    // Demander à l'utilisateur d'entrer les valeurs
    printf("Enter new threshold of the displayed edges (current: %.10lf): ", correlation_threshold);
    scanf("%lf", &correlation_threshold);

    printf("Entrez la taille initiale des noeuds (actuellement %.2f): ", initial_node_size);
    scanf("%f", &initial_node_size);

    printf("Entrez le coefficient de dilatation par rapport au degré (actuellement %.2f): ", degree_scale_factor);
    scanf("%f", &degree_scale_factor);
    printf("modeA =%d\n ",modeA);fflush(stdout);
    for (int i = 0; i < num_nodes; i++) {
        // Dessiner le cercle blanc autour du point
        if (modeA == 1) {if (S[i] == 1) 
        					{//printf("B%d ",S[i]); fflush(stdout); 
					glColor3f(1.0f, 0.0f, 0.0f);} 
				else {//printf("R%d ",S[i]); fflush(stdout); 
					glColor3f(0.0f, 0.0f, 1.0f);}
        				glPointSize(initial_node_size + node_degrees[i] * degree_scale_factor + 20.0f); // Taille légèrement supérieure
        				glBegin(GL_POINTS);
        				glVertex2f(positions[i].x, positions[i].y);
        				glEnd();} 
				else {glColor3f(1.0f, 1.0f, 1.0f); // Couleur blanche
        					glPointSize(initial_node_size + node_degrees[i] * degree_scale_factor + 5.0f); // Taille légèrement supérieure
        					glBegin(GL_POINTS);
        					glVertex2f(positions[i].x, positions[i].y);
        					glEnd();
					// Utiliser les couleurs des communautés pour les nœuds
        					int community = communities[i];
        					glColor3f(cluster_colors[community][0], cluster_colors[community][1], cluster_colors[community][2]);
        					glPointSize(initial_node_size + node_degrees[i] * degree_scale_factor);
        					glBegin(GL_POINTS);
        					glVertex2f(positions[i].x, positions[i].y);
        					glEnd();} 
        //printf("RR S[%d]=%d ",i,S[i]);fflush(stdout);
        
        

    }


//for (int i = 0; i < num_nodes; i++) {
    // Dessiner le cercle blanc autour du point
//    glColor3f(1.0f, 1.0f, 1.0f); // Couleur blanche pour le contour
//    float outer_radius = initial_node_size + node_degrees[i] * degree_scale_factor + 2.0f;
//    draw_circle(positions[i].x, positions[i].y, outer_radius);

    // Utiliser les couleurs des communautés pour les nœuds
//    int community = communities[i];
//    glColor3f(cluster_colors[community][0], cluster_colors[community][1], cluster_colors[community][2]);
//    float inner_radius = initial_node_size + node_degrees[i] * degree_scale_factor;
 //   draw_circle(positions[i].x, positions[i].y, inner_radius);
//}

    // Dessiner les arêtes comme des courbes de Bézier avec une couleur moyenne
    for (int i = 0; i < num_edges; i++) {
        if (edges[i].weight > correlation_threshold) {  
            Point p0 = positions[edges[i].node1];
            Point p1 = positions[edges[i].node2];

            double dx = fabs(p0.x - p1.x);
            double dy = fabs(p0.y - p1.y);

            if (dx < Lx / 2 && dy < Ly / 2) {
                // Calculer la couleur moyenne entre les deux communautés
                float avg_color[3];
                avg_color[0] = (cluster_colors[communities[edges[i].node1]][0] + cluster_colors[communities[edges[i].node2]][0]) / 2.0f;
                avg_color[1] = (cluster_colors[communities[edges[i].node1]][1] + cluster_colors[communities[edges[i].node2]][1]) / 2.0f;
                avg_color[2] = (cluster_colors[communities[edges[i].node1]][2] + cluster_colors[communities[edges[i].node2]][2]) / 2.0f;

                glColor3f(avg_color[0], avg_color[1], avg_color[2]);

                Point control;
                control.x = (p0.x + p1.x) / 2.0 + (p0.y - p1.y) * bez;
                control.y = (p0.y + p1.y) / 2.0 - (p0.x - p1.x) * bez;

                draw_bezier_curve(p0, p1, control, 20);  // Ajuster pour la douceur de la courbe
            }
        }
    }

    unsigned char *pixels = (unsigned char*)malloc(3 * width * height);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

    unsigned char *flipped_pixels = (unsigned char*)malloc(3 * width * height);
    for (int y = 0; y < height; y++) {
        memcpy(flipped_pixels + (height - 1 - y) * width * 3, pixels + y * width * 3, width * 3);
    }
    free(pixels);

    stbi_write_png("output.png", width, height, 3, flipped_pixels, width * 3);

    free(flipped_pixels);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteRenderbuffers(1, &depthBuffer);
    glDeleteRenderbuffers(1, &renderBuffer);
    glDeleteFramebuffers(1, &fbo);
}


// Générer un point aléatoire près du centre
void random_point_in_center(Point *p) {
    double center_width = Lx * 0.3;
    double center_height = Ly * 0.3;
    p->x = (rand() / (double)RAND_MAX) * center_width - center_width / 2;
    p->y = (rand() / (double)RAND_MAX) * center_height - center_height / 2;
}

// Calculer un vecteur avec un enroulement toroïdal
void toroidal_vector(Point *dir, Point p1, Point p2) {
    dir->x = p2.x - p1.x;
    dir->y = p2.y - p1.y;
    if (fabs(dir->x) > Lx / 2) dir->x -= copysign(Lx, dir->x);
    if (fabs(dir->y) > Ly / 2) dir->y -= copysign(Ly, dir->y);
}

// Calculer la distance toroïdale entre deux points
double toroidal_distance(Point p1, Point p2) {
    Point dir;
    toroidal_vector(&dir, p1, p2);
    return sqrt(dir.x * dir.x + dir.y * dir.y);
}

// chargement Data
void load_csv_data(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Could not open file %s\n", filename);
        exit(1);
    }

    char *line = NULL;
    size_t len = 0;

    // Première passe : Compter les lignes et les colonnes
    while (getline(&line, &len, file) != -1) {
        // Supprimer les retours à la ligne potentiels
        line[strcspn(line, "\r\n")] = '\0';
        num_rows++;

        // Compter les colonnes uniquement à la première ligne
        if (num_columns == 0) {
            int count = 1;  // Commence à 1 car il y a toujours une colonne avant la première virgule
            for (int i = 0; line[i] != '\0'; i++) {
                if (line[i] == ',') {
                    count++;
                }
            }
            num_columns = count;
        }
    }

    // Allouer de la mémoire pour les données
    data = (double **)malloc(num_rows * sizeof(double *));
    for (int i = 0; i < num_rows; i++) {
        data[i] = (double *)malloc(num_columns * sizeof(double));
    }

    // Deuxième passe : Lire les données
    rewind(file);
    int row = 0;
    while (getline(&line, &len, file) != -1) {
        line[strcspn(line, "\r\n")] = '\0';  // Supprimer les retours à la ligne potentiels

        int col = 0;
        char *start = line;
        char *end = NULL;

        // Parcourir chaque valeur séparée par des virgules
        while ((end = strchr(start, ',')) != NULL || *start != '\0') {
            if (end) {
                *end = '\0';  // Terminer la chaîne courante à la virgule
            }
            // Convertir la valeur en double
            data[row][col] = atof(start);
            col++;

            // Si end est NULL, on est à la dernière valeur
            if (!end) break;

            start = end + 1;  // Passer à la prochaine valeur
        }

        // Si le nombre de colonnes lues est différent de num_columns, avertir
        if (col != num_columns) {
            printf("Warning: Row %d has %d columns (expected %d).\n", row, col, num_columns);
        }

        row++;
    }
    
    fclose(file);
    free(line);  // Libérer la mémoire allouée par getline

    printf("Loaded CSV with %d rows and %d columns.\n", num_rows, num_columns);
    
        // Afficher les trois premières lignes
    printf("Les 3 premières lignes des données :\n");
    for (int i = 0; i < 5 && i < num_rows; i++) {
        for (int j = 0; j < num_columns; j++) {
            printf("%f ", data[i][j]);
        }
        printf("\n");
    }
}



// Fonction de comparaison pour qsort
int compare_double(const void *a, const void *b) {
    double arg1 = *(const double *)a;
    double arg2 = *(const double *)b;

    if (arg1 < arg2) return -1;
    if (arg1 > arg2) return 1;
    return 0;
}

// Fonction par dichotomie pour trouver le threshold
double calculate_threshold(int num_samples, int choice, int N) {
    double total_similarity = 0.0;
    int count = 0;

    // Allocation dynamique pour stocker les similarités
    double *similarities = (double *)malloc(num_samples * sizeof(double));
    if (similarities == NULL) {
        perror("Erreur d'allocation mémoire");
        exit(EXIT_FAILURE);
    }

    // Simulation du calcul de la similitude pour chaque échantillon
    for (int sample = 0; sample < num_samples; sample++) {
        int i = rand() % num_rows;
        int j = rand() % num_rows;
        if (i == j) continue;  // On évite de comparer un point avec lui-même

        // Calculer la similitude en fonction du choix de l'utilisateur
        double similarity = 0.0;
        switch (choice) {
            case 0:  // Corrélation
                similarity = correlation_similarity(i, j);
                break;
            case 1:  // Distance Cosinus
                similarity = cosine_similarity(i, j);
                break;
            case 2:  // Distance Euclidienne
                similarity = euclidean_distance(i, j);
                break;
            case 3:  // Norme L1
                similarity = L1_norm(i, j);
                break;
            case 4:  // Norme Linf
                similarity = Linf_norm(i, j);
                break;
            case 5:  // KL Div
                similarity = KL_divergence(i, j);
                break;
            default:
                printf("Choix non valide.\n");
                free(similarities);
                return 0.0;
        }

        similarities[count++] = similarity;
        total_similarity += (similarity);
    }
        printf("échantillon généré \n");
        fflush(stdout);

    // Tri des similarités pour la dichotomie
    qsort(similarities, num_samples, sizeof(double), compare_double);
    printf("tri Ok\n");
    fflush(stdout);
    // Utilisation de la dichotomie pour trouver le seuil
    double low = similarities[0];
    double high = similarities[num_samples - 1];
    double threshold = 0.5 * (low + high);
    //printf("%lf",threshold);
    while ((high-low) >= 0.000001) {
        // Réinitialiser le compteur d'arêtes pour le seuil actuel
        int current_edges = 0;
        for (int sample = 0; sample < num_samples; sample++) {
            if ((similarities[sample]) >= threshold) {
                current_edges++;
            }
        }

        // Comparer avec N et ajuster le seuil
        if (current_edges < N-100) {
            low = threshold;
            threshold = 0.5 * (threshold + high);
//printf("%lf",threshold);  // Dichotomie vers le haut
        } else if (current_edges > N+100) {
            high = threshold;
//printf("%lf",threshold);
            threshold = 0.5 * (low + threshold);  // Dichotomie vers le bas
        } else { printf("seuil trouvé \n");
	         fflush(stdout);
            break;  // On a trouvé le seuil exact
        }
    }

     low = similarities[0];
     high = similarities[num_samples - 1];
     double antiseuil = 0.5 * (low + high);
    //printf("%lf",antiseuil);
    while ((high-low) >= 0.000001) {
        // Réinitialiser le compteur d'arêtes pour le seuil actuel
        int current_edges = 0;
        for (int sample = 0; sample < num_samples; sample++) {
            if ((similarities[sample]) <= antiseuil) {
                current_edges++;
            }
        }

        // Comparer avec N et ajuster le seuil
        if (current_edges > N-100) {
            low = antiseuil;
            antiseuil = 0.5 * (antiseuil + high);
//printf("%lf",antiseuil);  // Dichotomie vers le haut
        } else if (current_edges < N+100) {
            high = antiseuil;
//printf("%lf",antiseuil);
            antiseuil = 0.5 * (low + antiseuil);  // Dichotomie vers le bas
        } else { printf("antiseuil trouvé \n");
	         fflush(stdout);
            break;  // On a trouvé le seuil exact
        }
    }

    // Afficher le seuil trouvé
      printf("Threshold for %d edges for %d nodes: %.6f\n", N, num_nodes, threshold);

    printf("AntiThreshold for %d edges for %d nodes: %.6f\n", N, num_nodes, antiseuil);

    free(similarities);  // Libérer la mémoire allouée
    return (count > 0) ? total_similarity / count : 0.0;
}



// Calculer la mesure de similarité moyenne sur un échantillon et générer un histogramme
double calculate_mean_similitude(int num_samples, int choice) {
    double total_similarity = 0.0;
    int count = 0;

    // Allocation dynamique pour stocker les similarités
    double *similarities = (double *)malloc(num_samples * sizeof(double));
    if (similarities == NULL) {
        perror("Erreur d'allocation mémoire");
        exit(EXIT_FAILURE);
    }

    // Initialiser l'histogramme
    int histogram[NUM_BINS] = {0};

    for (int sample = 0; sample < num_samples; sample++) {
        int i = rand() % num_rows;
        int j = rand() % num_rows;
        if (i == j) continue;  // On évite de comparer un point avec lui-même

        // Calculer la similitude en fonction du choix de l'utilisateur
        double similarity = 0.0;
        switch (choice) {
            case 0:  // Corrélation
                similarity = correlation_similarity(i, j);
                break;
            case 1:  // Distance Cosinus
                similarity = cosine_similarity(i, j);
                break;
            case 2:  // Distance Euclidienne
                similarity = euclidean_distance(i, j);
                break;
            case 3:  // Norme L1
                similarity = L1_norm(i, j);
                break;
            case 4:  // Norme Linf
                similarity = Linf_norm(i, j);
                break;
            case 5:  // KL Div
                similarity = KL_divergence(i, j);
                break;
            default:
                printf("Choix non valide.\n");
                free(similarities);
                return 0.0;
        }

        similarities[count++] = similarity;
        total_similarity += (similarity);  // Utilisation de la valeur absolue de la similitude

        // Calculer l'index du bin pour l'histogramme
        double scaled_similarity;
        if (choice == 0 || choice == 1) {  // Corrélation ou distance cosinus (bornée entre [-1, 1])
            scaled_similarity = (similarity + 1.0) / 2.0;  // Mise à l'échelle entre [0, 1]
        } else {  // Distances non bornées (euclidienne, L1, L∞)
            scaled_similarity = similarity;  // Pas de mise à l'échelle
        }
        int bin_index = (int)(scaled_similarity * NUM_BINS);
        if (bin_index >= NUM_BINS) bin_index = NUM_BINS - 1;
        if (bin_index < 0) bin_index = 0;
        histogram[bin_index]++;
    }

    // Normalisation des bins pour que leur somme soit égale à num_rows * (num_rows - 1) / 2
    int target_sum = num_rows * (num_rows - 1) / 2;
    int current_sum = 0;
    for (int bin = 0; bin < NUM_BINS; bin++) {
        current_sum += histogram[bin];
    }

    if (current_sum > 0) {
        double normalization_factor = (double)target_sum / current_sum;
        for (int bin = 0; bin < NUM_BINS; bin++) {
            histogram[bin] = (int)(histogram[bin] * normalization_factor);
        }
    }

    // Afficher l'histogramme normalisé
    printf("Histogram of Similarities (Normalized):\n");
    for (int bin = 0; bin < NUM_BINS; bin++) {
        double bin_start = (choice == 0 || choice == 1) ? -1.0 + (2.0 * bin) / NUM_BINS : (double)bin / NUM_BINS;
        double bin_end = (choice == 0 || choice == 1) ? -1.0 + (2.0 * (bin + 1)) / NUM_BINS : (double)(bin + 1) / NUM_BINS;
        printf("Bin [%.2f, %.2f): %d\n", bin_start, bin_end, histogram[bin]);
    }

    free(similarities);  // Libérer la mémoire allouée
    return (count > 0) ? total_similarity / count : 0.0;
}

/////////////////////////////////////
struct similarity_args {
    double threshold;
    double antiseuil;
    int row;
    int choice;
};

int incr_or_max(_Atomic int* n)
{
    int res = *n;
    while ( ! atomic_compare_exchange_weak(n, &res, res+1) )
    {
        if ( res >= MAX_EDGES ){
            *n = MAX_EDGES;
            return MAX_EDGES;
        }
    }

    return res;
}

void similarity_job(void * args){

    struct similarity_args * data = (struct similarity_args*) args;

    for ( int j = data->row + 1; j < num_rows; ++j)
    {
        double similarity = 0.0;

        switch (data->choice) {
            case 0:  // Corrélation
                similarity = correlation_similarity(data->row, j);
                break;
            case 1:  // Distance Cosinus
                similarity = cosine_similarity(data->row, j);
                break;
            case 2:  // Distance Euclidienne
                similarity = euclidean_distance(data->row, j);
                break;
            case 3:  // Norme L1
                similarity = L1_norm(data->row, j);
                break;
            case 4:  // Norme Linf
                similarity = Linf_norm(data->row, j);
                break;
            case 5:  // KL Div
                similarity = KL_divergence(data->row, j);
                break;
            default:
                printf("Choix non valide.\n");
        }
        
        if (similarity > data->threshold && num_edges < MAX_EDGES) {
            int edge_index = incr_or_max(&num_edges);
            if ( edge_index < MAX_EDGES ){
                edges[edge_index].node1 = data->row;
                edges[edge_index].node2 = j;
                edges[edge_index].weight = similarity;
            }
        } else if (similarity < data->antiseuil && num_antiedges < MAX_EDGES) {
            int antiedge_index = incr_or_max(&num_antiedges);
            if ( antiedge_index < MAX_EDGES ){
                antiedges[antiedge_index].node1 = data->row;
                antiedges[antiedge_index].node2 = j;
                antiedges[antiedge_index].weight = similarity;
            }
        }

    }
}

/////////////////////////////////////


// Fonction pour calculer la similitude en fonction du choix de l'utilisateur
void calculate_similitude_and_edges(double threshold, double antiseuil) {
    int choice;
    
    // Demander à l'utilisateur de choisir une mesure de similitude
    printf("Choisissez la mesure de similarité à utiliser:\n");
    printf("0: Corrélation\n");
    printf("1: Distance Cosinus\n");
    printf("2: Distance Euclidienne\n");
    printf("3: Norme L1\n");
    printf("4: Norme Linf\n");
    printf("5: KL divergence\n");
    scanf("%d", &choice);
    
    printf("Mean similitude: %.5lf\n",calculate_mean_similitude(5000,choice));
calculate_threshold(num_nodes,choice,10*num_nodes);
    printf("Enter up threshold: ");
    scanf("%lf", &threshold);
    while (getchar() != '\n');
    printf("Enter down threshold (coeff_antiarete): ");
    scanf("%lf", &antiseuil);
    while (getchar() != '\n');
    
    num_edges = 0;
    num_antiedges = 0;

    for (int i = 0; i < num_rows; i++) {
        struct similarity_args* s_args = (struct similarity_args*) malloc(sizeof(struct similarity_args));
        s_args->threshold = threshold;
        s_args->antiseuil = antiseuil;
        s_args->row       = i;
        s_args->choice    = choice;

        struct Job task;
        task.j = similarity_job;
        task.args = s_args;

        submit(&pool, task);
    }
    while ( GetOp(&pool) < num_rows ) {}


    printf("Nombre de nœuds : %d\n", num_rows);
    printf("Nombre d'arêtes : %d\n", num_edges);
    printf("Nombre d'anti-arêtes : %d\n", num_antiedges);
}

// Générer un point aléatoire dans le plan
void random_point_in_plane(Point *p) {
    p->x = (rand() / (double)RAND_MAX) * Lx - Lx / 2;
    p->y = (rand() / (double)RAND_MAX) * Ly - Ly / 2;
}

// Normaliser un point
void normalize(Point *p) {
    double norm = sqrt(p->x * p->x + p->y * p->y);
    if (norm > 0) {
        p->x /= norm;
        p->y /= norm;
    }
}

// Fonction pour initialiser les centres de clusters de manière aléatoire
void initialize_centers(){
    for (int i = 0; i < n_clusters; i++) {
        random_point_in_plane((Point *)centers[i]);
    }
}

// Assigner des noeuds aux clusters et mettre à jour les centres en utilisant l'algorithme k-means
void kmeans_iteration(Point *points, int num_points, int num_clusters, int *labels, double centers[][2], double Lx, double Ly) {
    int counts[MAX_NODES] = {0};

    // Modification pour utiliser moins de memoire et eviter un seg fault
    double ** new_centers = (double**) malloc(sizeof(double*) * num_clusters);  // Stocker les nouveaux centres calculés
    // TODO


    for (int i = 0; i < num_clusters; ++i)
    {
        new_centers[i] = (double*) malloc(sizeof(double) * 2);
        new_centers[i][0] = 0.;
        new_centers[i][1] = 1.;
    }

    // Assigner chaque point au cluster le plus proche et mettre à jour les centres
    for (int i = 0; i < num_points; i++) {
        double min_dist = DBL_MAX;
        int best_cluster = 0;

        for (int j = 0; j < num_clusters; j++) {
            Point dir;
            toroidal_vector(&dir, points[i], (Point){centers[j][0], centers[j][1]});
            double dist = (dir.x * dir.x + dir.y * dir.y);

            if (dist < min_dist) {
                min_dist = dist;
                best_cluster = j;
            }
        }

        labels[i] = best_cluster;

        // Ajuster les coordonnées du point pour qu'elles soient proches du centre du cluster
        double adjusted_x = points[i].x;
        double adjusted_y = points[i].y;

        while (adjusted_x - centers[best_cluster][0] > Lx / 2) adjusted_x -= Lx;
        while (centers[best_cluster][0] - adjusted_x > Lx / 2) adjusted_x += Lx;
        while (adjusted_y - centers[best_cluster][1] > Ly / 2) adjusted_y -= Ly;
        while (centers[best_cluster][1] - adjusted_y > Ly / 2) adjusted_y += Ly;

        new_centers[best_cluster][0] += adjusted_x;
        new_centers[best_cluster][1] += adjusted_y;
        counts[best_cluster]++;
    }

    // Mise à jour des centres de clusters en fonction des nouvelles assignations
    for (int i = 0; i < num_clusters; i++) {
        if (counts[i] > 0) {
            centers[i][0] = new_centers[i][0] / counts[i];
            centers[i][1] = new_centers[i][1] / counts[i];

            // Ramener les centres dans l'espace torique
            while (centers[i][0] < -Lx / 2) centers[i][0] += Lx;
            while (centers[i][0] > Lx / 2) centers[i][0] -= Lx;
            while (centers[i][1] < -Ly / 2) centers[i][1] += Ly;
            while (centers[i][1] > Ly / 2) centers[i][1] -= Ly;
        }
    }

    for (int i = 0; i < num_clusters; ++i)
    {
        free(new_centers[i]);
    }
    free(new_centers);
}

// Assigner des couleurs aux clusters
void assign_cluster_colors() {
    for (int i = 0; i < n_clusters; i++) {
        cluster_colors[i][0] = (float)rand() / RAND_MAX;
        cluster_colors[i][1] = (float)rand() / RAND_MAX;
        cluster_colors[i][2] = (float)rand() / RAND_MAX;
    }
}

// Mettre à jour les positions des noeuds en fonction des forces entre eux
void update_positions() {
   //int iteration;
    double Max_movement;
    double PasMaxX = Lx/10;
    double PasMaxY = Ly/10;
    double FMaxX = Lx/(friction*1000);
    double FMaxY = Ly/(friction*1000);
    double rep_force=1;
    thresholdS = (Lx/4000)*(Ly/4000);
    thresholdA = (Lx/4000)*(Ly/4000);
    epsilon = (Lx/800)*(Ly/800);
    seuilrep = (Lx/1000)*(Lx/1000);
    for (iteration = 0; iteration < max_iterations; iteration++) {
        Point forces[MAX_NODES] = {0};

        // Si une touche est enfoncée
        if (kbhit()) {
            char key = getchar();  // Récupérer la touche enfoncée
            if (key == 's' || key == 'S') {
                printf("Arrêt de la boucle, touche 's' enfoncée\n");
                iteration=max_iterations;  // Arrêt de la boucle si 's' est enfoncée
            }
        }


// Étape 1 : Forces d'attraction basées sur les arêtes
for (int edge_index = 0; edge_index < num_edges; edge_index++) {
    int node1 = edges[edge_index].node1;
    int node2 = edges[edge_index].node2;

    Point dir;
    toroidal_vector(&dir, positions[node1], positions[node2]);

    double dist_squared = dir.x * dir.x + dir.y * dir.y;
    double att_force = attraction_coeff; //*dist_squared;
    
    if (dist_squared > thresholdA) {            
    forces[node1].x += dir.x * att_force;
    forces[node1].y += dir.y * att_force;
    forces[node2].x -= dir.x * att_force;
    forces[node2].y -= dir.y * att_force;}
}


// Étape 2 : Forces de répulsion intra-cluster
double half_Lx = Lx / 2.0;
double half_Ly = Ly / 2.0;



for (int cluster = 0; cluster < n_clusters; cluster++) {
    int size = cluster_nodes[cluster].size;
    for (int i = 0; i < size; i++) {
        int node_i = cluster_nodes[cluster].nodes[i];



        Point pi = positions[node_i];
        for (int j = i + 1; j < size; j++) {
            int node_j = cluster_nodes[cluster].nodes[j];
            Point dir;
            toroidal_vector(&dir, pi, positions[node_j]);

            double dist_squared = dir.x * dir.x + dir.y * dir.y;
            if (dist_squared > seuilrep) { // Assume a minimum distance to avoid division by zero
                //double dist = sqrt(dist_squared);
                if (mode == 0) { rep_force = repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;} else
                   if (mode==1) { rep_force = repulsion_coeff/ dist_squared*dist_squared;} else
                        {if (communities[i] != communities[j]) {//printf("extra repulsion %d, %d \n",i,j);
                        rep_force = 100000*repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;} 
                        else {rep_force = repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;} }
                                    
                forces[node_i].x -= dir.x * rep_force;
                forces[node_i].y -= dir.y * rep_force;
                forces[node_j].x += dir.x * rep_force;
                forces[node_j].y += dir.y * rep_force;
            } else
          {double rep_force = repulsion_coeff / seuilrep;
                                    
                forces[node_i].x -= dir.x * rep_force;
                forces[node_i].y -= dir.y * rep_force;
                forces[node_j].x += dir.x * rep_force;
                forces[node_j].y += dir.y * rep_force;}
        }
            // capper les forces d'attractions
    forces[node_i].x = fmax(fmin(forces[node_i].x, FMaxX), -FMaxX);
    forces[node_i].y = fmax(fmin(forces[node_i].y, FMaxY), -FMaxY);

    }
}

        
        // Step 2bis: Repulsion forces based on anti-edges
for (int edge_index = 0; edge_index < num_antiedges; edge_index++) {
    int node1 = antiedges[edge_index].node1;
    int node2 = antiedges[edge_index].node2;

    Point dir;
    toroidal_vector(&dir, positions[node1], positions[node2]);

    double dist = sqrt(dir.x * dir.x + dir.y * dir.y);
    if (dist > seuilrep) {
        double rep_force = coeff_antiarete/(dist*dist);
        forces[node1].x -= (dir.x / dist) * rep_force;
        forces[node1].y -= (dir.y / dist) * rep_force;
        forces[node2].x += (dir.x / dist) * rep_force;
        forces[node2].y += (dir.y / dist) * rep_force;
    } else {double rep_force = coeff_antiarete/ seuilrep;
                                    
                forces[node1].x -= dir.x * rep_force;
                forces[node1].y -= dir.y * rep_force;
                forces[node2].x += dir.x * rep_force;
                forces[node2].y += dir.y * rep_force;}
}

// Étape 3 : Mettre à jour les positions en fonction des forces
Max_movement = 0.0;
for (int i = 0; i < num_nodes; i++) {
    velocities[i].x = (velocities[i].x + forces[i].x) * friction;
    velocities[i].y = (velocities[i].y + forces[i].y) * friction;
    velocities[i].x = fmin(fmax(velocities[i].x, -PasMaxX), PasMaxX); // Capper la force en x à 1
    velocities[i].y = fmin(fmax(velocities[i].y, -PasMaxY), PasMaxY); // Capper la force en y à 1

    positions[i].x += velocities[i].x;
    positions[i].y += velocities[i].y;
                // Appliquer les conditions aux limites toroïdales
            while (positions[i].x < -half_Lx) positions[i].x += Lx;
            while (positions[i].x > half_Lx) positions[i].x -= Lx;
            while (positions[i].y < -half_Ly) positions[i].y += Ly;
            while (positions[i].y > half_Ly) positions[i].y -= Ly;

    Max_movement = fmax(Max_movement, velocities[i].x * velocities[i].x + velocities[i].y * velocities[i].y);
}

        // Étape 4 : Mettre à jour les clusters tous les quelques itérations
        if (iteration % (saut * (1+0*espacement)) == 0) {
            espacement++;
            centers_converged = 0;

            while (centers_converged == 0) {
                double old_centers[MAX_NODES][2];
                memcpy(old_centers, centers, sizeof(centers));

                kmeans_iteration(positions, num_nodes, n_clusters, clusters, centers, Lx, Ly);

                centers_converged = 1;
                for (int i = 0; i < n_clusters; i++) {
                    double dx = centers[i][0] - old_centers[i][0];
                    double dy = centers[i][1] - old_centers[i][1];
                    if ((dx * dx + dy * dy) > epsilon) {
                        centers_converged = 0;
                        break;
                    }
                }
            }
            clear_clusters();

            for (int i = 0; i < num_nodes; i++) {
                int best_cluster = clusters[i];
                add_node_to_cluster(best_cluster, i);
            }
        }

        //display();
	//printf("Iteration %d, Max_movement %lf, friction %lf \n", iteration, Max_movement,friction);
	//fflush(stdout);  // Force le vidage du tampon de la sortie standard

	  if (Max_movement==Max_movementOld) {friction *= 0.7;}
          Max_movementOld=Max_movement;
          if ((Max_movement < thresholdS && iteration > 50) || iteration > max_iterations-1 ) {
            printf("Paused at iteration %d due to small movement or stop.\n", iteration);

            char user_input;
            while (1) {
printf("Simulation paused.\n Press 'c' to continue \n 'q' to quit \n 'm' to modify parameters \n 'k' to redo communities search \n 's' to save the image \n 'r' to save the graph \n 't' to translate \n 'd' pour charger un fichier .dot\n  ? ");

user_input = getchar();

if (user_input == 'c') {
    printf("Enter new friction (current: %.10lf): ", friction);
    scanf("%lf", &friction);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix

    printf("Enter new saut (current: %d): ", saut);
    scanf("%d", &saut);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix

    printf("Enter new amortissement (current: %.10lf): ", amortissement);
    scanf("%lf", &amortissement);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix

    printf("Enter new threshold (current: %.10lf): ", thresholdS);
    scanf("%lf", &thresholdS);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix
    
    printf("Resuming simulation...\n");
    espacement=1;
    break;
} else if (user_input == 'r') {
    espacement=1;
    printf("Enregistrer l'image, la position des points et le fichier GEXF.\n");

    // Ouvrir un fichier pour enregistrer les positions des points
    FILE *output_file = fopen("positions.txt", "w");
    if (output_file != NULL) {
        for (int i = 0; i < num_nodes; i++) {
            fprintf(output_file, "Node %d: Position (%f, %f), Communauté %d, [", 
                    i, positions[i].x, positions[i].y, communities[i]);
                           for (int j = 0; j < num_columns; j++) {
            fprintf(output_file, " %f, ", 
                    data[i][j] );
        }
        fprintf(output_file, " ]\n ");
        }
        fclose(output_file);
        printf("Positions saved in 'positions.txt'.\n");
    } else {
        printf("Error: Unable to write to 'positions.txt'.\n");
    }

    // Ouvrir un fichier pour enregistrer le fichier GEXF
    FILE *gexf_file = fopen("graph.gexf", "w");
    if (gexf_file != NULL) {
        // Écriture de l'en-tête XML et des métadonnées avec déclaration du namespace 'viz'
        fprintf(gexf_file, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fprintf(gexf_file, "<gexf xmlns=\"http://www.gexf.net/1.2draft\" xmlns:viz=\"http://www.gexf.net/1.2draft/viz\" version=\"1.2\">\n");
        fprintf(gexf_file, "  <meta lastmodifieddate=\"%s\">\n", __DATE__);
        fprintf(gexf_file, "    <creator>ForceAtlas Simulation</creator>\n");
        fprintf(gexf_file, "    <description>A graph generated by ForceAtlas simulation</description>\n");
        fprintf(gexf_file, "  </meta>\n");

        // Section des nœuds
        fprintf(gexf_file, "  <graph mode=\"static\" defaultedgetype=\"undirected\">\n");
        fprintf(gexf_file, "    <nodes>\n");
        for (int i = 0; i < num_nodes; i++) {
            fprintf(gexf_file, "      <node id=\"%d\" label=\"Node %d\">\n", i, i);
            fprintf(gexf_file, "        <viz:position x=\"%f\" y=\"%f\" z=\"0.0\" />\n", positions[i].x, positions[i].y);
            fprintf(gexf_file, "        <viz:color r=\"%d\" g=\"%d\" b=\"%d\" a=\"1\" />\n",
                    (int)(cluster_colors[clusters[i]][0] * 255),
                    (int)(cluster_colors[clusters[i]][1] * 255),
                    (int)(cluster_colors[clusters[i]][2] * 255));
            fprintf(gexf_file, "      </node>\n");
        }
        fprintf(gexf_file, "    </nodes>\n");

        // Section des arêtes
        fprintf(gexf_file, "    <edges>\n");
        for (int i = 0; i < num_edges; i++) {
            fprintf(gexf_file, "      <edge id=\"%d\" source=\"%d\" target=\"%d\" weight=\"%f\" />\n", 
                    i, edges[i].node1, edges[i].node2, edges[i].weight);
        }
        fprintf(gexf_file, "    </edges>\n");
        fprintf(gexf_file, "  </graph>\n");
        fprintf(gexf_file, "</gexf>\n");

        fclose(gexf_file);
        printf("GEXF graph saved in 'graph.gexf'.\n");
    } else {
        printf("Error: Unable to write to 'graph.gexf'.\n");
    }



    break;
}

else if (user_input == 's') {
    printf("Entrer courbure (current: %f): ", bez);
    scanf("%f", &bez);
    printf("Saving image.\n");

    save_image_opengl(IMAGE_SIZE, IMAGE_SIZE);
    printf("Image saved as 'output.png'.\n");

    // Réinitialisation d'OpenGL pour continuer la simulation
    glViewport(0, 0, 800, 800);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-Lx / 2, Lx / 2, -Ly / 2, Ly / 2);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    break;
}


else if (user_input == 'd') {
    printf("Entrer chemin d'accès: ");
    
    // Use scanf with a width specifier to avoid buffer overflow
    scanf("%255s", filename);  // Limiting the input to 255 characters

    // Initialize your variables and call parse_dot_file
    num_nodes = 0;
    num_edges = 0;
    num_antiedges = 0;

    // Call your function to parse the file
    parse_dot_file(filename);

    printf("\n Graph Loaded, %d nodes, %d edges \n",num_nodes, num_edges);
    fflush(stdout);

    // Ensure `num_nodes` is positive
    if (num_nodes <= 0) {
        fprintf(stderr, "Error: Number of nodes is invalid.\n");
        break;
    }
    // Check if the number of nodes exceeds the maximum allowed nodes
if (num_nodes > MAX_NODES) {
    fprintf(stderr, "Warning: Number of nodes exceeds the maximum capacity of %d. Reducing to maximum allowed nodes.\n", MAX_NODES);
    num_nodes = MAX_NODES;
}

    printf("Début Louvain \n");
num_communities = louvain_method();
    initialize_community_colors();
    printf("Louvain fini \n");
    
    // Initialize positions and velocities arrays
    printf("Initialize positions and velocities arrays \n");
    for (int i = 0; i < num_nodes; i++) {
        random_point_in_center(&positions[i]);
        velocities[i].x = velocities[i].y = 0.0;
    }
    printf("Initialization OK \n");
    

    printf("Initialisation clusters \n");
    n_clusters = (int)sqrt(num_nodes);
    init_clusters(n_clusters);
    initialize_centers();
    assign_cluster_colors();
    printf("Clusters OK \n");

    printf("Calcul des degrés \n");
    calculate_node_degrees();
    printf("Degré OK \n");
    break;
}


else if (user_input == 'q') {
    printf("Quitting simulation.\n");
    free_clusters(); // Free allocated memory before exit
    exit(0);
}

else if (user_input == 't') {
    double dx, dy;
    double center_x = 0.0;
    double center_y = 0.0;

    // Sum up all x and y coordinates
    for (int i = 0; i < num_nodes; i++) {
        center_x += positions[i].x;
        center_y += positions[i].y;
    }

    // Divide by the number of points to get the average
    center_x /= num_nodes;
    center_y /= num_nodes;
    printf("Enter the translation vector (dx, dy), actuellement le barycentre est en (%f,%f):\n",center_x,center_y);
    scanf("%lf %lf", &dx, &dy);
    
    translate_positions(dx, dy);
    printf("Translation applied: (%.2lf, %.2lf)\n", dx, dy);
    break;
}

 else if (user_input == 'm') {
// Demander les nouveaux paramètres
       printf("Enter new mode (current: %d): repulsion by degree (0), repulsion by edges (1), repulsion by communities (2)   ", mode);
    scanf("%d", &mode);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix
        printf("Enter new threshold of the displayed edges (current: %.10lf): ", correlation_threshold);
    scanf("%lf", &correlation_threshold);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix
    // Demander les nouvelles dimensions du tore
    
    printf("Enter new x dimension (current: %lf): ", Lx);
    scanf("%lf", &Lx);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix
    
    printf("Enter new y dimension (current: %lf): ", Ly);
    scanf("%lf", &Ly);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix
    
    thresholdA = (Lx/4000)*(Ly/4000);
    PasMaxX = Lx/10;
    PasMaxY = Ly/10;
    FMaxX = Lx/(friction*1000);
    FMaxY = Ly/(friction*1000);
    thresholdS = (Lx/4000)*(Ly/4000);
    epsilon = (Lx/800)*(Ly/800);

    // Demander le nouveau nombre de clusters
    int new_n_clusters;
    printf("Enter new number of clusters (current: %d): ", n_clusters);
    scanf("%d", &new_n_clusters);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix
    // Vérification que le nombre de clusters est positif et différent de l'actuel
    if (new_n_clusters > 0 ) {
        // Libérer la mémoire allouée pour les anciens clusters
        free_clusters();

        // Mettre à jour le nombre de clusters
        n_clusters = new_n_clusters;

        // Réinitialiser les clusters avec le nouveau nombre de clusters
        init_clusters(n_clusters); // Initialize clusters with dynamic arrays
        initialize_centers(); // Réinitialiser les centres des clusters
        assign_cluster_colors(); // Réassigner les couleurs des clusters

        printf("Number of clusters updated to %d.\n", n_clusters);
    } else {
        printf("Invalid number of clusters. Please enter a positive number.\n");
    }
    printf("Enter new repulsion force (current: %.10lf): ", repulsion_coeff);
    scanf("%lf", &repulsion_coeff);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix
    
        printf("Enter new repulsion threshold (current: %.10lf): ", seuilrep);
    scanf("%lf", &seuilrep);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix 

    printf("Enter new attraction force (current: %.10lf): ", attraction_coeff);
    scanf("%lf", &attraction_coeff);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix

    
            printf("Enter new attraction threshold (current: %.10lf): ", thresholdA);
    scanf("%lf", &thresholdA);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix

        printf("Enter new antiedges force (current: %.10lf): ", coeff_antiarete);
    scanf("%lf", &coeff_antiarete);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix


    printf("Enter new friction (current: %.10lf): ", friction);
    scanf("%lf", &friction);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix

    printf("Enter new amortissement (current: %.10lf): ", amortissement);
    scanf("%lf", &amortissement);
    while (getchar() != '\n')
    ; // Vider le tampon après chaque choix

    // Mettre à jour les autres paramètres
    printf("Parameters updated.\n");
    glViewport(0, 0, 800, 800);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-Lx / 2, Lx / 2, -Ly / 2, Ly / 2);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    break;
}
else if (user_input == 'k') {
    printf("select Louvain component (0), Louvain component (1), Leiden (2), Leiden CPM (3), couleurs à partir d'un fichier (4)\n");
    scanf("%d", &ll);

if (ll == 1) {modeA=0;
    num_communities = louvain_methodC();
    initialize_community_colors();
    printf("Louvain Composante fini \n");
} else if (ll == 2) {modeA=0;
    num_communities = leiden_method();
    initialize_community_colors();
    printf("Leiden fini \n");
} else if (ll == 0) {modeA=0;
    num_communities = louvain_method();
    initialize_community_colors();
    printf("Leiden fini \n");
} else if (ll == 3) {modeA=0;
    printf("résolution CPM (current %lf) \n",lambda);
    scanf("%lf", &lambda);
    num_communities = leiden_method_CPM();
    initialize_community_colors();
    printf("Leiden CPM fini \n");
}else if (ll == 4) {
    //int nbValeurs;
    //int S[MAX_NODES]={0};
    // Demander le chemin du fichier à l'utilisateur
    lireColonneCSV(S, &nbValeurs);
    // Afficher les valeurs lues
    printf("nombres de valeurs lues : %d pour %d données \n",nbValeurs,num_nodes);
    modeA =1;
    compute_ratio_S(S);
}  else {
    printf("Option invalide\n");
}
    break;
} else {
    printf("Invalid input. Press 'c' to continue, 'q' to quit, 'm' to modify parameters, or 'k' to redo k-means: ");
}

                while ((getchar()) != '\n')
                ; // Clear the buffer
            }
        }

        friction *= amortissement;
    }
}

// Fonction d'affichage pour OpenGL
void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    // Définir l'épaisseur des arêtes (par exemple, 1.0 pour une ligne fine)
    glLineWidth(0.1f);  // Changez cette valeur pour ajuster l'épaisseur des arêtes

    // Dessiner les arêtes comme des lignes droites avec la couleur moyenne des communautés
    for (int i = 0; i < num_edges; i++) {
        Point p0 = positions[edges[i].node1];
        Point p1 = positions[edges[i].node2];

        if (edges[i].weight > correlation_threshold) {  
            // Calculer les différences en x et y
            double dx = fabs(p0.x - p1.x);
            double dy = fabs(p0.y - p1.y);

            // Condition pour vérifier si les différences sont dans les limites spécifiées
            if (dx < Lx / 2 && dy < Ly / 2) {
                // Calculer la couleur moyenne entre les deux communautés
                float avg_color[3];
                avg_color[0] = (cluster_colors[communities[edges[i].node1]][0] + cluster_colors[communities[edges[i].node2]][0]) / 2.0f;
                avg_color[1] = (cluster_colors[communities[edges[i].node1]][1] + cluster_colors[communities[edges[i].node2]][1]) / 2.0f;
                avg_color[2] = (cluster_colors[communities[edges[i].node1]][2] + cluster_colors[communities[edges[i].node2]][2]) / 2.0f;

                // Définir la couleur à la couleur moyenne
                glColor3f(avg_color[0], avg_color[1], avg_color[2]);

                // Dessiner la ligne droite
                glBegin(GL_LINES);
                glVertex2f(p0.x, p0.y);
                glVertex2f(p1.x, p1.y);
                glEnd();
            }
        }
    }

    // Afficher les nœuds
    for (int i = 0; i < num_nodes; i++) {
        int community = communities[i];
        glColor3f(cluster_colors[community][0], cluster_colors[community][1], cluster_colors[community][2]);
        glPointSize(0.1f + node_degrees[i] * 0.05f);

        glBegin(GL_POINTS);
        glVertex2f(positions[i].x, positions[i].y);
        glEnd();
    }

    // Optionnel : Afficher les centres des communautés
    // glPointSize(10.0f);
    // glBegin(GL_POINTS);
    // for (int i = 0; i < num_nodes; i++) {
    //     int community = communities[i];
    //     glColor3f(cluster_colors[community][0], cluster_colors[community][1], cluster_colors[community][2]);
    //     glVertex2f(centers[i][0], centers[i][1]);
    // }
    // glEnd();

    glutSwapBuffers();
}


// Fonction d'idle pour OpenGL
void idle() {
    if (!pause_simulation) {
        update_positions();
        glutPostRedisplay();
    }
}

/**
 * 
 * 
 * 
 Fonction Pour L'interface
 * 
 * 
 * 
 * 
 */

 JNIEXPORT void JNICALL Java_graph_Graph_updatePositions
 (JNIEnv * env, jobject obj)
{
   //int iteration;
   double Max_movement;
   double PasMaxX = Lx/10;
   double PasMaxY = Ly/10;
   double FMaxX = Lx/(friction*1000);
   double FMaxY = Ly/(friction*1000);
   double rep_force=1;
   thresholdS = (Lx/4000)*(Ly/4000);
   thresholdA = (Lx/4000)*(Ly/4000);
   epsilon = (Lx/800)*(Ly/800);
   seuilrep = (Lx/1000)*(Lx/1000);

   static Point forces[MAX_NODES] = {0};

    for (int edge_index = 0; edge_index < num_edges; edge_index++) {
        int node1 = edges[edge_index].node1;
        int node2 = edges[edge_index].node2;

        Point dir;
        toroidal_vector(&dir, positions[node1], positions[node2]);

        double dist_squared = dir.x * dir.x + dir.y * dir.y;
        double att_force = attraction_coeff; //*dist_squared;
        
        if (dist_squared > thresholdA) {            
            forces[node1].x += dir.x * att_force;
            forces[node1].y += dir.y * att_force;
            forces[node2].x -= dir.x * att_force;
            forces[node2].y -= dir.y * att_force;
        }
    }

    double half_Lx = Lx / 2.0;
    double half_Ly = Ly / 2.0;
    
    
    for (int cluster = 0; cluster < n_clusters; cluster++) {
        int size = cluster_nodes[cluster].size;
        for (int i = 0; i < size; i++) {
            int node_i = cluster_nodes[cluster].nodes[i];
    
    
    
            Point pi = positions[node_i];
            for (int j = i + 1; j < size; j++) {
                int node_j = cluster_nodes[cluster].nodes[j];
                Point dir;
                toroidal_vector(&dir, pi, positions[node_j]);
    
                double dist_squared = dir.x * dir.x + dir.y * dir.y;
                if (dist_squared > seuilrep) { // Assume a minimum distance to avoid division by zero
                    //double dist = sqrt(dist_squared);
                    if (mode == 0) { rep_force = repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;} else
                       if (mode==1) { rep_force = repulsion_coeff/ dist_squared*dist_squared;} else
                            {if (communities[i] != communities[j]) {//printf("extra repulsion %d, %d \n",i,j);
                            rep_force = 100000*repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;} 
                            else {rep_force = repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;} }
                                        
                    forces[node_i].x -= dir.x * rep_force;
                    forces[node_i].y -= dir.y * rep_force;
                    forces[node_j].x += dir.x * rep_force;
                    forces[node_j].y += dir.y * rep_force;
                } else
              {double rep_force = repulsion_coeff / seuilrep;
                                        
                    forces[node_i].x -= dir.x * rep_force;
                    forces[node_i].y -= dir.y * rep_force;
                    forces[node_j].x += dir.x * rep_force;
                    forces[node_j].y += dir.y * rep_force;}
            }
                // capper les forces d'attractions
        forces[node_i].x = fmax(fmin(forces[node_i].x, FMaxX), -FMaxX);
        forces[node_i].y = fmax(fmin(forces[node_i].y, FMaxY), -FMaxY);
    
        }
    }

    // Step 2bis: Repulsion forces based on anti-edges
    for (int edge_index = 0; edge_index < num_antiedges; edge_index++) {
        int node1 = antiedges[edge_index].node1;
        int node2 = antiedges[edge_index].node2;

        Point dir;
        toroidal_vector(&dir, positions[node1], positions[node2]);

        double dist = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (dist > seuilrep) {
            double rep_force = coeff_antiarete/(dist*dist);
            forces[node1].x -= (dir.x / dist) * rep_force;
            forces[node1].y -= (dir.y / dist) * rep_force;
            forces[node2].x += (dir.x / dist) * rep_force;
            forces[node2].y += (dir.y / dist) * rep_force;
        } else {double rep_force = coeff_antiarete/ seuilrep;
                                        
                    forces[node1].x -= dir.x * rep_force;
                    forces[node1].y -= dir.y * rep_force;
                    forces[node2].x += dir.x * rep_force;
                    forces[node2].y += dir.y * rep_force;}
    }

    // Étape 3 : Mettre à jour les positions en fonction des forces
    Max_movement = 0.0;
    for (int i = 0; i < num_nodes; i++) {
        velocities[i].x = (velocities[i].x + forces[i].x) * friction;
        velocities[i].y = (velocities[i].y + forces[i].y) * friction;
        velocities[i].x = fmin(fmax(velocities[i].x, -PasMaxX), PasMaxX); // Capper la force en x à 1
        velocities[i].y = fmin(fmax(velocities[i].y, -PasMaxY), PasMaxY); // Capper la force en y à 1

        positions[i].x += velocities[i].x;
        positions[i].y += velocities[i].y;
                    // Appliquer les conditions aux limites toroïdales
                while (positions[i].x < -half_Lx) positions[i].x += Lx;
                while (positions[i].x > half_Lx) positions[i].x -= Lx;
                while (positions[i].y < -half_Ly) positions[i].y += Ly;
                while (positions[i].y > half_Ly) positions[i].y -= Ly;

        Max_movement = fmax(Max_movement, velocities[i].x * velocities[i].x + velocities[i].y * velocities[i].y);
    }

        // Étape 4 : Mettre à jour les clusters tous les quelques itérations
        if (iteration % (saut * (1+0*espacement)) == 0) {
            espacement++;
            centers_converged = 0;

            while (centers_converged == 0) {
                double old_centers[MAX_NODES][2];
                memcpy(old_centers, centers, sizeof(centers));

                kmeans_iteration(positions, num_nodes, n_clusters, clusters, centers, Lx, Ly);

                centers_converged = 1;
                for (int i = 0; i < n_clusters; i++) {
                    double dx = centers[i][0] - old_centers[i][0];
                    double dy = centers[i][1] - old_centers[i][1];
                    if ((dx * dx + dy * dy) > epsilon) {
                        centers_converged = 0;
                        break;
                    }
                }
            }
            clear_clusters();

            for (int i = 0; i < num_nodes; i++) {
                int best_cluster = clusters[i];
                add_node_to_cluster(best_cluster, i);
            }
        }


}

JNIEXPORT jintArray JNICALL Java_graph_Graph_getCommunitites
  (JNIEnv * env, jobject obj)
{
    jintArray result = (*env)->NewIntArray(env, MAX_NODES);

    (*env)->SetIntArrayRegion(env, result, 0, MAX_NODES, communities);

    return result;
}

JNIEXPORT jobjectArray JNICALL Java_graph_Graph_getClusterColors
  (JNIEnv * env, jobject obj)
{
    jclass obj_class = (*env)->FindClass(env, "[F");
    jobjectArray result = (*env)->NewObjectArray(env, MAX_NODES, obj_class, 0);

    for (int i = 0; i < MAX_NODES; ++i)
    {
        jfloatArray float_array = (*env)->NewFloatArray(env, 3);
        (*env)->SetFloatArrayRegion(env, float_array, 0, 3, cluster_colors[i]);

        (*env)->SetObjectArrayElement(env, result, i, float_array);
    }

    return result;
}

JNIEXPORT jobjectArray JNICALL Java_graph_Graph_getEdges
  (JNIEnv * env, jobject obj)
{
    // remplacer "backendinterface/Edge" par "[packageName]/[nomClasse]"
    jclass obj_class = (*env)->FindClass(env, "graph/EdgeInterm");
    jmethodID edge_constructor = (*env)->GetMethodID(env, obj_class, "<init>", "(IID)V");
    jobject initial_elem = (*env)->NewObject(env, obj_class, edge_constructor, 0, 0, 0.);
    
    jobjectArray result = (*env)->NewObjectArray(env, num_edges, obj_class, initial_elem);

    for (int i = 0; i < num_edges; ++i)
    {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;
        double weight = edges[i].weight;
        jobject edge = (*env)->NewObject(env, obj_class, edge_constructor, node1, node2, weight);
    
        (*env)->SetObjectArrayElement(env, result, i, edge);
    }

    return result;
}

JNIEXPORT jobjectArray JNICALL Java_graph_Graph_getPositions
  (JNIEnv * env, jobject obj)
{
    // remplacer "backendinterface/Point" par "[packageName]/[nomClasse]"
    jclass obj_class = (*env)->FindClass(env, "graph/Vertex");
    jmethodID point_constructor = (*env)->GetMethodID(env, obj_class, "<init>", "(DD)V");
    jobject initial_elem = (*env)->NewObject(env, obj_class, point_constructor, 0., 0.);

    jobjectArray result = (*env)->NewObjectArray(env, num_nodes, obj_class, initial_elem);

    for (int i = 0; i < num_nodes; ++i)
    {
        double x = positions[i].x;
        double y = positions[i].y;

        jobject point = (*env)->NewObject(env, obj_class, point_constructor, x, y);

        (*env)->SetObjectArrayElement(env, result, i, point);
    }

    return result;
}

JNIEXPORT void JNICALL Java_graph_Graph_startsProgram
  (JNIEnv * env, jobject obj, jstring filepath)
{
    srand(time(NULL));

    const char* str = (*env)->GetStringUTFChars(env, filepath, JNI_FALSE);

    load_csv_data(str);
    
    InitPool(&pool, 1000, 8);

    int *sampled_rows = NULL;
    int sampled_num_rows = 0;
    sample_rows(&sampled_rows, &sampled_num_rows);
    num_rows = sampled_num_rows;
    num_nodes = num_rows;
    #ifdef _DEBUG_
        struct chrono chr;
        chr_assign_log(&chr, "debug.csv");
        chr_start_clock(&chr);
        calculate_similitude_and_edges(threshold, coeff_antiarete);
        chr_stop(&chr);
        chr_close_log(&chr);
    #else
        calculate_similitude_and_edges(threshold, coeff_antiarete);
    #endif


    
    printf("Louvain (0), Louvain par composante (1) ou Leiden (2) ou Leiden CPM (3) ou couleurs speciales (4) \n");
    scanf("%d", &ll);


    if (ll == 0) {modeA=0;
        num_communities = louvain_method();
        initialize_community_colors();
        printf("Louvain fini \n");
    } else if (ll == 1) {modeA=0;
        num_communities = louvain_methodC();
        initialize_community_colors();
        printf("Louvain fini \n");
    } else if (ll == 2) {modeA=0;
        num_communities = leiden_method();
        initialize_community_colors();
        printf("Leiden fini \n");
    } else if (ll == 3) {modeA=0;
        num_communities = leiden_method_CPM();
        initialize_community_colors();
        printf("Leiden CPM fini \n");
    } else if (ll == 4) {
        //int nbValeurs;
        //int S[MAX_NODES]={0};
        num_communities = leiden_method_CPM();
        initialize_community_colors();
        // Demander le chemin du fichier à l'utilisateur
        lireColonneCSV(S, &nbValeurs);
        // Afficher les valeurs lues
        printf("nombres de valeurs lues : %d pour %d données\n",nbValeurs,num_nodes);
        modeA =1;
        compute_ratio_S(S);
    } else {
        printf("Option invalide\n");
    }

    compute_average_vectors();
    printf("Vecteurs moyens fini \n");
    for (int i = 0; i < num_nodes; i++) {
        random_point_in_center(&positions[i]);
        velocities[i].x = velocities[i].y = 0.0;
    }
    n_clusters = (int)sqrt(num_nodes);
    init_clusters(n_clusters);
    initialize_centers();
    assign_cluster_colors();
    calculate_node_degrees();

    free(sampled_rows);
}

JNIEXPORT void JNICALL Java_graph_Graph_freeAllocatedMemory
  (JNIEnv * env, jobject obj)
{

    // Libérer la mémoire allouée pour les voisins
    for (int i = 0; i < num_nodes; i++) {
    	Neighbor* neighbor = adjacency_list[i].head;
        while (neighbor != NULL) {
           Neighbor* next = neighbor->next;
           free(neighbor);
           neighbor = next;
        }
    }
    free_clusters();

    FreePool(&pool);

}

////////////////////////////////////////////////////////////////////////////////////////////////


// Fonction principale
int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <csv_file>\n", argv[0]);
        return 1;
    }

    srand(time(NULL));

    load_csv_data(argv[1]);
    
    InitPool(&pool, 1000, 8);

    int *sampled_rows = NULL;
    int sampled_num_rows = 0;
    sample_rows(&sampled_rows, &sampled_num_rows);
    num_rows = sampled_num_rows;
    num_nodes = num_rows;
    #ifdef _DEBUG_
        struct chrono chr;
        chr_assign_log(&chr, "debug.csv");
        chr_start_clock(&chr);
        calculate_similitude_and_edges(threshold, coeff_antiarete);
        chr_stop(&chr);
        chr_close_log(&chr);
    #else
        calculate_similitude_and_edges(threshold, coeff_antiarete);
    #endif


    
printf("Louvain (0), Louvain par composante (1) ou Leiden (2) ou Leiden CPM (3) ou couleurs speciales (4) \n");
scanf("%d", &ll);


if (ll == 0) {modeA=0;
    num_communities = louvain_method();
    initialize_community_colors();
    printf("Louvain fini \n");
} else if (ll == 1) {modeA=0;
    num_communities = louvain_methodC();
    initialize_community_colors();
    printf("Louvain fini \n");
} else if (ll == 2) {modeA=0;
    num_communities = leiden_method();
    initialize_community_colors();
    printf("Leiden fini \n");
} else if (ll == 3) {modeA=0;
    num_communities = leiden_method_CPM();
    initialize_community_colors();
    printf("Leiden CPM fini \n");
} else if (ll == 4) {
    //int nbValeurs;
    //int S[MAX_NODES]={0};
    num_communities = leiden_method_CPM();
    initialize_community_colors();
    // Demander le chemin du fichier à l'utilisateur
    lireColonneCSV(S, &nbValeurs);
    // Afficher les valeurs lues
    printf("nombres de valeurs lues : %d pour %d données\n",nbValeurs,num_nodes);
    modeA =1;
    compute_ratio_S(S);
} else {
    printf("Option invalide\n");
}

    compute_average_vectors();
    printf("Vecteurs moyens fini \n");
    for (int i = 0; i < num_nodes; i++) {
        random_point_in_center(&positions[i]);
        velocities[i].x = velocities[i].y = 0.0;
    }
    n_clusters = (int)sqrt(num_nodes);
    init_clusters(n_clusters);
    initialize_centers();
    assign_cluster_colors();
    calculate_node_degrees();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Force-Directed Graph Layout");

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-Lx / 2, Lx / 2, -Ly / 2, Ly / 2);

    glutDisplayFunc(display);
    glutIdleFunc(idle);
  // glutKeyboardFunc(keyboard);


    glutMainLoop();

    // Libérer la mémoire allouée pour les voisins
    for (int i = 0; i < num_nodes; i++) {
    	Neighbor* neighbor = adjacency_list[i].head;
        while (neighbor != NULL) {
           Neighbor* next = neighbor->next;
           free(neighbor);
           neighbor = next;
    }
}
    free_clusters();
    free(sampled_rows);

    FreePool(&pool);
    return 0;
}
