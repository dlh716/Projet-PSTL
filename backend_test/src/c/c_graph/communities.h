#ifndef FORCEATLAS_COMMUNITIES_H
#define FORCEATLAS_COMMUNITIES_H

#include "cluster.h"

typedef struct {
    Neighbor* head;
} AdjacencyList;

typedef struct {
    int community;
    double total_weight;
    int component;  // Ajout du champ pour la composante connexe
} Community;

int louvain_method();
int louvain_methodC();
int leiden_method_CPM();
int leiden_method();
int count_unique_communities(int *communities, int num_nodes);
void find_connected_components();
void mark_component(int, int);
void apply_louvain_to_component(int);
void initialize_adjacency_list();
void initialize_community_colors();
void compute_average_vectors();
void add_edge_to_adjacency_list(int node, int neighbor, double weight);
double calculate_gain_modularity(int node, int new_community, double total_graph_weight); 
double calculate_gain_modularity_cpm(int node, int new_community, double resolution_parameter);

#endif