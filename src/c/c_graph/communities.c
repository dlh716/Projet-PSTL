#include "communities.h"
#include "../global.h"

double lambda=0.1;
AdjacencyList adjacency_list[MAX_NODES];
Community node_community_map[MAX_NODES];
int num_communities=0;
int component_sizes[MAX_NODES]; // Tableau pour stocker la taille de chaque composante
int num_components = 0;

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

void initialize_adjacency_list() {
    for (int i = 0; i < MAX_NODES; i++) {
        adjacency_list[i].head = NULL;  // Initialiser chaque tête de liste à NULL
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

// Fonction pour ajouter un voisin dans la liste d'adjacence
void add_edge_to_adjacency_list(int node, int neighbor, double weight) {
    Neighbor* new_neighbor = (Neighbor*)malloc(sizeof(Neighbor));
    new_neighbor->node = neighbor;
    new_neighbor->weight = weight;
    new_neighbor->next = adjacency_list[node].head;
    adjacency_list[node].head = new_neighbor;
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