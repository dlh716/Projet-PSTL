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

#include "c_graph/cluster.h"
#include "c_graph/graph.h"
#include "../concurrent/Pool.h"

#ifdef _DEBUG_
    #include "../debug/debug_time.h"
#endif

#include "../../out/graph_Graph.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../lib/stb_image_write.h"

#define MAX_LINE_LENGTH 10000  // Longueur maximale d'une ligne dans le fichier
#define IMAGE_SIZE 15000
#define NUM_BINS 100  // Nombre de bins pour l'histogramme
#define MAX_SAMPLES 120000  // Maximum number of rows to sample
#define EPSILON 1e-12  // Pour éviter la division par 0
// Define the number of segments to approximate the circle (the higher the number, the smoother the circle)
#define NUM_SEGMENTS 50

// Variables globales pour stocker les données de la simulation
char filename[1000];
int modeA=0; //mode pour afficher des noeuds en fonction de classe
float bez=0.2;
double lambda=0.1;
int num_components = 0;
int nbValeurs;
double amortissement = 0.999;
double Max_movementOld=0;
int pause_simulation = 0;
int max_iterations = 5000;
double **data = NULL;  // Stocker les données CSV
int num_rows = 0, num_columns = 0;

char delimiter[1] = "\0";

short pause_updates = 0;
int mode_similitude;
struct Pool pool;

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
void calculate_similitude_and_edges(int mode_similitude, double threshold, double antiseuil);
void random_point_in_plane(Point *p);
void normalize(Point *p);
void update_positions(void);
void kmeans_iteration(Point *points, int num_points, int num_clusters, int *labels, double centers[][2], double Lx, double Ly);
void display(void);
void idle(void);
void calculate_node_degrees(void);
void save_image(const char *filename, int width, int height);
void draw_bezier_curve(Point p0, Point p1, Point control, int segments);
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
short str_is_number(char* line);

short str_is_number(char* line)
{
    if ( *line == '-' ) {
        ++line;
    } else if ( *line == '\0' ) {
        return 0;
    }

    short pt = 0;
    while ( (*line >= 48 && *line <= 57) || (*line == 46 && ! pt) || *line == 101 ) {
        if (*line == 46)
            pt = 1;
        else if (*line == 101)
            return str_is_number(++line);
        ++line;
    }

    return *line == '\0';
}


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
        char *token = strtok(ligne, delimiter);
        if ( token != NULL && str_is_number(token) ) {
            // Vérifier si le tableau S n'est pas plein
            if (*nbValeurs < MAX_NODES) {
                // Convertir la chaîne en entier et stocker dans le tableau S
                if (str_is_number(token)) {
                    S[*nbValeurs] = atoi(token);
                } else {
                    S[*nbValeurs] = 0;
                }
                //printf("S[%d]=%d ",*nbValeurs,S[*nbValeurs]);
                (*nbValeurs)++;
            } else {
                printf("Nombre maximum de valeurs atteint.\n");
                break;
            }
        } else {
            
            printf("Missing values in csv file");
        }
    }
    // Afficher les trois premières lignes
    printf("Les 5 premières lignes des données :\n");
    for (int i = 0; i < 5 && i < num_rows; i++) {
            printf("%d \n", S[i]);
    }
    fclose(fichier);
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



// Générer un point aléatoire près du centre
void random_point_in_center(Point *p) {
    double center_width = Lx * 0.3;
    double center_height = Ly * 0.3;
    p->x = (rand() / (double)RAND_MAX) * center_width - center_width / 2;
    p->y = (rand() / (double)RAND_MAX) * center_height - center_height / 2;
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
            int count = 0;  // Commence à 1 car il y a toujours une colonne avant la première virgule
            for (int i = 0; line[i] != '\0'; i++) {
                if (delimiter[0] == '\0' || line[i] == ',' || line[i] == ';' || line[i] == '|' || line[i] == ' ' || line[i] == '\t') {
                    delimiter[0] = line[i];
                    count++;
                } else if (line[i] == delimiter[0]) {
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
        while ((end = strchr(start, delimiter[0])) != NULL || *start != '\0') {
            if (end) {
                *end = '\0';  // Terminer la chaîne courante à la virgule
            }
            // Convertir la valeur en double
            if ( ! str_is_number(start) ){
                printf("Warning %s: Missing value on row %d, col %d\n", start, row, col);
            }
            data[row][col] = atof(start);
            col++;

            // Si end est NULL, on est à la dernière valeur
            if (!end) break;

            start = end + 1;  // Passer à la prochaine valeur
        }

        // Si le nombre de colonnes lues est différent de num_columns, avertir
        if (col != num_columns) {
            //printf("Warning: Row %d has %d columns (expected %d).\n", row, col, num_columns);
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
double calculate_threshold(int num_samples, int choice, int N, double* threshold, double* anti_threshold) {
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
    // Utilisation de la dichotomie pour trouver le seuil
    double low = similarities[0];
    double high = similarities[num_samples - 1];
    *threshold = 0.5 * (low + high);
    // TODO Potentiel boucle infini ici
    while ((high-low) >= 0.000001) {
        // Réinitialiser le compteur d'arêtes pour le seuil actuel
        int current_edges = 0;
        for (int sample = 0; sample < num_samples; sample++) {
            if ((similarities[sample]) >= *threshold) {
                current_edges++;
            }
        }

        // Comparer avec N et ajuster le seuil
        if (current_edges < N-100) {
            low = *threshold;
            *threshold = 0.5 * (*threshold + high);
        } else if (current_edges > N+100) {
            high = *threshold;
            *threshold = 0.5 * (low + *threshold);  // Dichotomie vers le bas
        } else { printf("seuil trouvé \n");
	         fflush(stdout);
            break;  // On a trouvé le seuil exact
        }
    }

     low = similarities[0];
     high = similarities[num_samples - 1];
     *anti_threshold = 0.5 * (low + high);
    // TODO Potentiel boucle infini ici
    while ((high-low) >= 0.000001) {
        // Réinitialiser le compteur d'arêtes pour le seuil actuel
        int current_edges = 0;
        for (int sample = 0; sample < num_samples; sample++) {
            if ((similarities[sample]) <= *anti_threshold) {
                current_edges++;
            }
        }

        // Comparer avec N et ajuster le seuil
        if (current_edges > N-100) {
            low = *anti_threshold;
            *anti_threshold = 0.5 * (*anti_threshold + high);
//printf("%lf",antiseuil);  // Dichotomie vers le haut
        } else if (current_edges < N+100) {
            high = *anti_threshold;
//printf("%lf",antiseuil);
            *anti_threshold = 0.5 * (low + *anti_threshold);  // Dichotomie vers le bas
        } else {
	        fflush(stdout);
            break;  // On a trouvé le seuil exact
        }
    }

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

struct mean_similitude_args {
    int choice;
    int thread_id;
    int start_row;
    int end_row;
    double *res;
    int *cpt;
    int *histogram;
    int **thread_histograms;
};

void mean_similitude_job(void *args) {
    struct mean_similitude_args *data = (struct mean_similitude_args*) args;
    double somme = 0.0;
    int count = 0;
    int *local_histogram = data->thread_histograms[data->thread_id];
    printf("Thread %d row %d à %d\n", data->thread_id, data->start_row, data->end_row);
    for (int i = data->start_row; i < data->end_row; i++) {
        for (int j = i + 1; j < num_rows; j++) {
            double similarity = 0.0;
            switch (data->choice) {
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
                    similarity = 0.0;
                    break; 
            }
            if(!isnan(similarity)){
                somme += similarity;
            }
            count++;

            double scaled_similarity = (data->choice == 0 || data->choice == 1) 
                ? (similarity + 1.0) / 2.0 
                : similarity;

            int bin_index = (int)(scaled_similarity * NUM_BINS);
            if (bin_index >= NUM_BINS) bin_index = NUM_BINS - 1;
            if (bin_index < 0) bin_index = 0;
            local_histogram[bin_index]++;
        }
    }

    data->res[data->thread_id] = somme;
    data->cpt[data->thread_id] = count;
    printf("Thread %d somme = %f count = %d\n", data->thread_id, somme, count);
}

double calculate_mean_similitude_paralel(int choice) {
    int num_threads = pool.nb_threads;
    double *res = calloc(num_threads, sizeof(double));
    int *cpt = calloc(num_threads, sizeof(int));
    int histogram[NUM_BINS] = {0};

    int **thread_histograms = calloc(num_threads, sizeof(int *));
    for (int t = 0; t < num_threads; t++) {
        thread_histograms[t] = calloc(NUM_BINS, sizeof(int));
    }

    int rows_per_thread = num_rows / num_threads;
    int remaining_rows = num_rows % num_threads;

    for (int t = 0; t < num_threads; t++) {
        struct mean_similitude_args *args = malloc(sizeof(struct mean_similitude_args));
        args->choice = choice;
        args->thread_id = t;
        args->start_row = t * rows_per_thread;
        args->end_row = (t + 1) * rows_per_thread;
        args->res = res;
        args->cpt = cpt;
        args->histogram = histogram;
        args->thread_histograms = thread_histograms;

        if (t == num_threads - 1) {
            args->end_row += remaining_rows;
        }

        struct Job task;
        task.j = mean_similitude_job;
        task.args = args;
        submit(&pool, task);
    }

    while (GetOp(&pool) < num_threads) {}

    for (int t = 0; t < num_threads; t++) {
        for (int bin = 0; bin < NUM_BINS; bin++) {
            histogram[bin] += thread_histograms[t][bin];
        }
    }


    double somme = 0.0;
    int count = 0;
    for (int t = 0; t < num_threads; t++) {
        somme += res[t];
        count += cpt[t];
    }


    printf("Histogram of Similarities (Normalized):\n");
    for (int bin = 0; bin < NUM_BINS; bin++) {
        double bin_start = (choice == 0 || choice == 1) ? -1.0 + (2.0 * bin) / NUM_BINS : (double)bin / NUM_BINS;
        double bin_end = (choice == 0 || choice == 1) ? -1.0 + (2.0 * (bin + 1)) / NUM_BINS : (double)(bin + 1) / NUM_BINS;
        printf("Bin [%.2f, %.2f): %d\n", bin_start, bin_end, histogram[bin]);
    }

    return (count > 0) ? somme / count : 0.0;
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
void calculate_similitude_and_edges(int md, double threshold, double antiseuil) {
        
    num_edges = 0;
    num_antiedges = 0;

    for (int i = 0; i < num_rows; i++) {
        struct similarity_args* s_args = (struct similarity_args*) malloc(sizeof(struct similarity_args));
        s_args->threshold = threshold;
        s_args->antiseuil = antiseuil;
        s_args->row       = i;
        s_args->choice    = md;

        struct Job task;
        task.j = similarity_job;
        task.args = s_args;

        submit(&pool, task);
    }
    while ( GetOp(&pool) < num_rows ) {}

}

// Normaliser un point
void normalize(Point *p) {
    double norm = sqrt(p->x * p->x + p->y * p->y);
    if (norm > 0) {
        p->x /= norm;
        p->y /= norm;
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

JNIEXPORT jboolean JNICALL Java_graph_Graph_updatePositions
 (JNIEnv * env, jobject obj)
{
   double Max_movementOld = 0.0;
   double Max_movement;
   double FMaxX = Lx/(friction*1000);
   double FMaxY = Ly/(friction*1000);
   thresholdS = (Lx/4000)*(Ly/4000);
   thresholdA = (Lx/4000)*(Ly/4000);
   epsilon = (Lx/800)*(Ly/800);
   seuilrep = (Lx/1000)*(Lx/1000);

   double PasMaxX = Lx / 10.;
   double PasMaxY = Ly / 10.;

   static int iteration = 0;
   static Point forces[MAX_NODES] = {0};

   if ( pause_updates == 0 ){
        repulsion_edges(forces);
        repulsion_intra_clusters(forces, FMaxX, FMaxY);
        repulsion_anti_edges(forces);
        Max_movement = update_position_forces(forces, PasMaxX, PasMaxY, Max_movement);
        update_clusters();

        ++iteration;

        if (Max_movement==Max_movementOld) {
            friction *= 0.7;
        }
           
        Max_movementOld=Max_movement;
        friction *= amortissement;
   }

   if ((Max_movement < thresholdS && iteration > 50) || iteration > max_iterations-1 ) {
        // end state reached
        pause_updates = 1;

        return 0;
   }

   return 1;
}

JNIEXPORT jintArray JNICALL Java_graph_Graph_getCommunities
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
    jclass obj_class = (*env)->FindClass(env, "graph/EdgeC");
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

JNIEXPORT jobject JNICALL Java_graph_Graph_startsProgram
  (JNIEnv * env, jobject obj, jstring filepath)
{
    srand(time(NULL));

    const char* str = (*env)->GetStringUTFChars(env, filepath, JNI_FALSE);

    load_csv_data(str);

    jclass obj_class = (*env)->FindClass(env, "[D");
    jobjectArray result = (*env)->NewObjectArray(env, num_rows, obj_class, 0);

    for (int i = 0; i < num_rows; ++i)
    {
        jdoubleArray double_array = (*env)->NewDoubleArray(env, num_columns);
        (*env)->SetDoubleArrayRegion(env, double_array, 0, num_columns, data[i]);

        (*env)->SetObjectArrayElement(env, result, i, double_array);
    }

    return result;

}

JNIEXPORT jobject JNICALL Java_graph_Graph_computeThreshold
  (JNIEnv * env, jobject obj, jint modeSimilitude)
{

    InitPool(&pool, 1000, 8);

    int *sampled_rows = NULL;
    int sampled_num_rows = 0;
    sample_rows(&sampled_rows, &sampled_num_rows);
    num_rows = sampled_num_rows;
    num_nodes = num_rows;
    
    double threshold, antiseuil;
    printf("Mean similitude: %.5lf\n",calculate_mean_similitude_paralel(modeSimilitude));
    mode_similitude = modeSimilitude;
    calculate_threshold(num_nodes, modeSimilitude, 10*num_nodes, &threshold, &antiseuil);

    jclass res_class = (*env)->FindClass(env, "graph/Metadata");
    jmethodID constructor = (*env)->GetMethodID(env, res_class, "<init>", "(IDD)V");
    jobject res = (*env)->NewObject(env, res_class, constructor, num_nodes, threshold, antiseuil);

    free(sampled_rows);

    return res;

}


JNIEXPORT jobject JNICALL Java_graph_Graph_initiliazeGraph
  (JNIEnv *env, jobject obj, jint md, jdouble thresh, jdouble anti_thresh)
{

    printf("Mode de similitude utilisé : %d\n", mode_similitude);
    printf("Threshold utilisé : %lf\n", thresh);
    printf("Anti-Threshold utilisé : %lf\n", anti_thresh);

    #ifdef _DEBUG_
        struct chrono chr;
        chr_assign_log(&chr, "debug.csv");
        chr_start_clock(&chr);
        calculate_similitude_and_edges(mode_similitude, thresh, anti_thresh);
        chr_stop(&chr);
        chr_close_log(&chr);
    #else
        calculate_similitude_and_edges(mode_similitude, thresh, anti_thresh);
    #endif

    modeA=0;
    if (md == 0) {
        num_communities = louvain_method();
    } else if (md == 1) {
        num_communities = louvain_methodC();
    } else if (md == 2) {
        num_communities = leiden_method();
    } else if (md == 3) {
        num_communities = leiden_method_CPM();
    } else if (md == 4) {
        // TODO 
        //int nbValeurs;
        //int S[MAX_NODES]={0};
        num_communities = leiden_method_CPM();
        // Demander le chemin du fichier à l'utilisateur
        lireColonneCSV(S, &nbValeurs);
        // Afficher les valeurs lues
        printf("nombres de valeurs lues : %d pour %d données\n",nbValeurs,num_nodes);
        modeA =1;
        compute_ratio_S(S);
    } else {
        printf("Option invalide\n");
    }
    initialize_community_colors();

    #ifdef _DEBUG_
        printf("Fin Calcul Communaute");
    #endif

    compute_average_vectors();

    #ifdef _DEBUG_
        printf("Vecteurs moyens fini \n");
    #endif

    for (int i = 0; i < num_nodes; i++) {
        random_point_in_center(&positions[i]);
        velocities[i].x = velocities[i].y = 0.0;
    }
    n_clusters = (int)sqrt(num_nodes);
    init_clusters(n_clusters);
    initialize_centers();
    assign_cluster_colors();
    calculate_node_degrees();

    jclass res_class = (*env)->FindClass(env, "graph/Metadata");
    jmethodID constructor = (*env)->GetMethodID(env, res_class, "<init>", "(IDDIII)V");
    jobject res = (*env)->NewObject(env, res_class, constructor, num_nodes, thresh, anti_thresh, num_edges, num_antiedges, n_clusters);

    return res;   

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

JNIEXPORT void JNICALL Java_graph_Graph_setSaut
  (JNIEnv * env, jobject obj, jint s)
{
    saut = s;
    espacement = 1;
}

JNIEXPORT void JNICALL Java_graph_Graph_setThresholdS
  (JNIEnv * env, jobject obj, jdouble thresh)
{
    thresholdS = thresh;
}

JNIEXPORT void JNICALL Java_graph_Graph_setFriction
  (JNIEnv * env, jobject obj, jdouble f)
{
    friction = f;
}

JNIEXPORT void JNICALL Java_graph_Graph_setModeRepulsion
  (JNIEnv * env, jobject obj, jint modeRepulsion)
{
    mode = modeRepulsion;
}

JNIEXPORT void JNICALL Java_graph_Graph_setAntiRepulsion
  (JNIEnv * env, jobject obj, jdouble repulsion)
{
    coeff_antiarete = repulsion;
}

JNIEXPORT void JNICALL Java_graph_Graph_setAttractionCoeff
  (JNIEnv * env, jobject obj, jdouble coeff)
{
    attraction_coeff = coeff;
}

JNIEXPORT void JNICALL Java_graph_Graph_setThresholdA
  (JNIEnv * env, jobject obj, jdouble thresh)
{
    thresholdA = thresh;
}

JNIEXPORT void JNICALL Java_graph_Graph_setSeuilRep
  (JNIEnv * env, jobject obj, jdouble seuil)
{
    seuilrep = seuil;
}

JNIEXPORT void JNICALL Java_graph_Graph_setDimension
  (JNIEnv * env, jobject obj, jdouble width, jdouble height)
{
    Lx = width;
    Ly = height;
}

JNIEXPORT void JNICALL Java_graph_Graph_setAmortissement
  (JNIEnv * env, jobject obj, jdouble amort)
{
    amortissement = amort;
}

JNIEXPORT void JNICALL Java_graph_Graph_setNodePosition
  (JNIEnv * env, jobject obj, jint index, jdouble x, jdouble y)
{
    positions[index].x = x;
    positions[index].y = y;
}