#ifndef FORCEATLAS_CLUSTER_H
#define FORCEATLAS_CLUSTER_H

#include <string.h>
#include <stdlib.h>
#include <float.h>
#include "graph.h"

typedef struct {
    int *nodes;  // Tableau dynamique de noeuds dans le cluster
    int size;    // Nombre de noeuds actuels dans le cluster
    int capacity; // Capacité actuelle du tableau
} Cluster;

typedef struct Neighbor {
    int node;
    double weight;
    struct Neighbor* next;
} Neighbor;

void kmeans_iteration(int num_points, int num_clusters, int *labels, double centers[][2], double Lx, double Ly);

void update_clusters();
void repulsion_intra_clusters(Point* forces, double FMaxX, double FMaxY);

// Fonction pour initialiser les centres de clusters de manière aléatoire
void initialize_centers();
// Vider les clusters
void clear_clusters();
// Réinitialiser les clusters si le nombre de clusters change
void reinitialize_clusters(int new_num_clusters);
// Libérer la mémoire allouée pour les clusters
void free_clusters();
// Assigner des couleurs aux clusters
void assign_cluster_colors();
// Initialiser les clusters avec des tableaux dynamiques
void init_clusters(int num_clusters);
// Ajouter un noeud à un cluster, redimensionner si nécessaire
void add_node_to_cluster(int cluster_id, int node);


#endif

